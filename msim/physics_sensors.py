"""Shared MEMS noise and position-domain GNSS; physical-time rolling LiDAR."""
import copy
import hashlib
from importlib.resources import files
import json
import math
from pathlib import Path
import random
import numpy as np
from .imu_model import ImuConfig, ImuNoiseModel
from .bullet_backend import rotation


def sensor_profile(path=None,lidar=None):
    root=files('msim').joinpath('data')
    p=json.loads(Path(path).read_text(encoding='utf-8') if path else root.joinpath('sensors.json').read_text(encoding='utf-8'))
    if not path or lidar:
        lidar=lidar or 'vlp16'
        presets=json.loads(root.joinpath('lidars.json').read_text(encoding='utf-8'))
        old_pattern=p['lidar'].get('pattern_file')
        for key in ('pattern_file','pattern_sha256','point_rate_hz','scan_pattern'):
            p['lidar'].pop(key,None)
        p['lidar'].update(presets[lidar]);p['models']['lidar']=lidar
        p['lidar'].setdefault('scan_pattern','uniform')
        if lidar=='mid360':
            candidate=Path(__file__).resolve().parents[1]/'adapters/gazebo_classic/data/livox/mid360.csv'
            p['lidar']['pattern_file']=old_pattern or str(candidate)
    ImuConfig.from_dict(p['imu_ros'])
    if p['imu_ros']['rate_hz']>500:raise ValueError('IMU rate exceeds the 500 Hz physical stepping rate')
    l=p['lidar'];g=p['gps']
    for k in ('horizontal_samples','vertical_samples'):
        if type(l.get(k)) is not int or l[k]<2:raise ValueError('Invalid LiDAR '+k)
    if l['horizontal_samples']*l['vertical_samples']>500_000:raise ValueError('LiDAR frame exceeds 500,000 rays')
    for k in ('rate_hz','range_resolution_m','range_min_m','range_max_m'):
        if not isinstance(l.get(k),(int,float)) or not math.isfinite(l[k]) or l[k]<=0:raise ValueError('Invalid LiDAR '+k)
    if not 0<l['range_min_m']<l['range_max_m'] or l.get('timing') not in ('rolling','snapshot'):raise ValueError('Invalid LiDAR range/timing')
    if not -90<l['vertical_min_deg']<l['vertical_max_deg']<90:raise ValueError('Invalid beam elevations')
    if not math.isfinite(l['range_noise_sigma_m']) or l['range_noise_sigma_m']<0:raise ValueError('Invalid range noise')
    if g.get('native_noise') or g.get('adaptive_environment'):
        raise ValueError('Bullet requires explicit position-domain GNSS; Gazebo native noise/obstruction cannot be silently emulated')
    for k in ('position_sigma_enu_m','bias_sigma_enu_m','bias_tau_enu_s'):
        if k not in g or len(g[k])!=3 or not np.isfinite(g[k]).all() or min(g[k])<0:raise ValueError('Invalid GPS ENU vector '+k)
    if min(g['bias_tau_enu_s'])<=0:raise ValueError('GPS correlation times must be positive')
    expected=np.square(g['position_sigma_enu_m'])+np.square(g['bias_sigma_enu_m'])
    if not np.allclose(expected,g.get('reported_variance_enu_m2',[]),rtol=1e-10,atol=1e-12):raise ValueError('GPS covariance differs from noise model')
    return copy.deepcopy(p)


class GnssModel:
    """Same OU transition/marginal covariance as receiver_position_error.hpp.

    Python/C++ normal generators differ; distributions, not sequences, match.
    """
    def __init__(self,config):
        self.config=config;self.rng=random.Random(config['seed']);self.bias=None;self.last=None
    def sample(self,stamp_ns,position):
        if self.last is not None and stamp_ns<=self.last:raise ValueError('GPS sample time must increase')
        dt=0 if self.last is None else (stamp_ns-self.last)/1e9
        g=self.config;noise=[];bias=[]
        for i in range(3):
            phi=0 if self.bias is None else math.exp(-dt/g['bias_tau_enu_s'][i])
            b=phi*(0 if self.bias is None else self.bias[i])+g['bias_sigma_enu_m'][i]*math.sqrt(1-phi*phi)*self.rng.gauss(0,1)
            bias.append(b);noise.append(b+g['position_sigma_enu_m'][i]*self.rng.gauss(0,1))
        self.bias=bias;self.last=stamp_ns
        return np.array(position)+noise


class RollingLidar:
    def __init__(self,config,step_s):
        self.cfg=config;self.step_s=step_s;self.period_ns=round(1e9/config['rate_hz'])
        if abs(self.period_ns/1e9/step_s-round(self.period_ns/1e9/step_s))>1e-7:
            raise ValueError('LiDAR period must be an integer number of physics steps')
        self.columns=config['horizontal_samples'];self.lines=config['vertical_samples']
        self.start_ns=0;self.column=0;self.buffers=[];self.rng=np.random.default_rng(config['seed']);self.scan_index=0
        self.pattern=None;pattern=config.get('scan_pattern','uniform')
        if pattern=='livox_csv':
            path=Path(config.get('pattern_file',''))
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=config.get('pattern_sha256'):
                raise ValueError('Mid-360 needs the pinned external scan CSV; set pattern_file in --sensor-profile')
            raw=np.loadtxt(path,delimiter=',',skiprows=1,usecols=(1,2))
            az=np.deg2rad(raw[:,0]);el=np.deg2rad(90-raw[:,1])
            self.pattern=np.column_stack([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)])
        elif pattern!='uniform':raise ValueError('Unsupported LiDAR pattern')
        self.elevations=np.deg2rad(np.linspace(config['vertical_min_deg'],config['vertical_max_deg'],self.lines))

    def _cast(self,backend,count,stamp):
        columns=np.arange(self.column,self.column+count);rings=np.tile(np.arange(self.lines,dtype=np.uint16),count)
        if self.pattern is None:
            az=np.repeat(-np.pi+2*np.pi*columns/self.columns,self.lines);el=np.tile(self.elevations,count)
            directions=np.column_stack([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)])
        else:
            index=(self.scan_index*self.columns*self.lines+self.column*self.lines+np.arange(count*self.lines))%len(self.pattern)
            directions=self.pattern[index]
        cfg=self.cfg;ranges=backend.rays(directions,minimum=cfg['range_min_m'],maximum=cfg['range_max_m'])
        valid=np.isfinite(ranges)&(ranges>cfg['range_min_m'])&(ranges<cfg['range_max_m']-1e-6)
        ranges=ranges[valid]+self.rng.normal(0,cfg['range_noise_sigma_m'],int(valid.sum()))
        ranges=np.round(ranges/cfg['range_resolution_m'])*cfg['range_resolution_m']
        ok=(ranges>cfg['range_min_m'])&(ranges<cfg['range_max_m'])
        points=directions[valid][ok]*ranges[ok,None]
        offset=(stamp-self.start_ns)/1e9 if cfg['timing']=='rolling' else 1e-6
        state=backend.state();R=rotation(state['orientation'])
        world=points@R.T+state['position']+R@np.array([0,0,.15])
        self.buffers.append((points.astype('f4'),rings[valid][ok],np.full(len(points),offset,dtype='f4'),world.astype('f4')))
        self.column+=count

    def _finish(self,end_ns):
        xyz=np.concatenate([b[0] for b in self.buffers]);ring=np.concatenate([b[1] for b in self.buffers]);times=np.concatenate([b[2] for b in self.buffers])
        order=np.argsort(ring,kind='stable') if self.pattern is None else np.arange(len(ring))
        fields={k:xyz[order,i] for i,k in enumerate(('x','y','z'))}
        fields.update(ring=ring[order],time=times[order],intensity=np.zeros(len(xyz),dtype='f4'))
        return dict(stamp_ns=self.start_ns,end_ns=end_ns,fields=fields,columns=self.column,
                    world_points=np.concatenate([b[3] for b in self.buffers])[order],
                    note='XYZ in acquisition-time lidar frame; time seconds from frame start')

    def update(self,backend):
        now=backend.state()['stamp_ns'];ready=None
        if self.cfg['timing']=='snapshot':
            if self.column and now-self.start_ns<self.period_ns:return None
            self.start_ns=now;self.column=0;self.buffers=[];self._cast(backend,self.columns,now)
            ready=self._finish(now);self.scan_index+=1;return ready
        if now-self.start_ns>=self.period_ns:
            if self.column!=self.columns:raise ValueError('Incomplete physical LiDAR frame')
            ready=self._finish(now);self.scan_index+=1;self.start_ns=now;self.column=0;self.buffers=[]
        due=min(self.columns,math.ceil(((now-self.start_ns)/1e9+self.step_s)*self.cfg['rate_hz']*self.columns-1e-7))
        if due>self.column:self._cast(backend,due-self.column,now)
        return ready


def imu_sample(model,state,gravity):
    R=rotation(state['orientation']);lever=np.array([0,0,.02]);w=state['angular_velocity']
    acceleration=R.T@(state['acceleration']-gravity)+np.cross(state['angular_acceleration'],lever)+np.cross(w,np.cross(w,lever))
    return model.sample(state['stamp_ns']/1e9,w,acceleration,state['orientation'])
