"""Resolve explicit, reproducible research sensor parameter presets.

LiDAR presets approximate beam geometry, not calibrated vendor emulators.
"""
import copy
import json
import math
import hashlib
from pathlib import Path

BASE = Path(__file__).parent / 'config/sensor_profile.json'
DEFAULT_GPS = 'f9p-dgnss-ucr'
GPS_REFERENCE = json.loads((BASE.parent/'gps_references/f9p_dgnss_ucr_2023.json').read_text())
MID360_PATTERN = Path(__file__).resolve().parents[2] / 'gazebo_classic/data/livox/mid360.csv'
MID360_SHA256 = 'aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a'
LIDARS = {
    'vlp16': dict(vertical_samples=16, vertical_min_deg=-15., vertical_max_deg=15.,
                  horizontal_samples=1800, range_min_m=.3, range_max_m=100.),
    'hdl32': dict(vertical_samples=32, vertical_min_deg=-30.67, vertical_max_deg=10.67,
                  horizontal_samples=1800, range_min_m=.3, range_max_m=100.),
    'os1-64': dict(vertical_samples=64, vertical_min_deg=-22.5, vertical_max_deg=22.5,
                   horizontal_samples=1024, range_min_m=.3, range_max_m=100.),
    'mid360': dict(vertical_samples=4, vertical_min_deg=-7., vertical_max_deg=52.,
                   horizontal_samples=5000, rate_hz=10., range_min_m=.1, range_max_m=40.,
                   scan_pattern='livox_csv', point_rate_hz=200000.,
                   pattern_file=str(MID360_PATTERN), pattern_sha256=MID360_SHA256),
}
IMUS = ('mems-baseline', 'low-noise')
GPS = {
    DEFAULT_GPS: dict(position_sigma_xy_m=0., position_sigma_z_m=0.,
                     position_sigma_enu_m=[GPS_REFERENCE['table'][axis]['white_sigma_m'] for axis in ('east','north','down')],
                     bias_sigma_enu_m=[math.sqrt(GPS_REFERENCE['table'][axis]['bias_variance_m2']) for axis in ('east','north','down')],
                     bias_tau_enu_s=[GPS_REFERENCE['table'][axis]['correlation_time_s'] for axis in ('east','north','down')],
                     velocity_sigma_xy_mps=0., velocity_sigma_z_mps=0.,
                     reported_variance_mode='model_marginal',
                     native_noise=False, adaptive_environment=False,
                     environment_range_m=80., environment_noise_gain=0.),
    'standalone': dict(position_sigma_xy_m=.1, position_sigma_z_m=.15,
                       bias_sigma_xy_m=.7, bias_sigma_z_m=2., bias_tau_s=120.,
                       velocity_sigma_xy_mps=.05, velocity_sigma_z_mps=.1,
                       reported_variance_xy_m2=1., reported_variance_z_m2=5.,
                       native_noise=True, adaptive_environment=True,
                       environment_range_m=80., environment_noise_gain=3.),
    'native': dict(position_sigma_xy_m=0., position_sigma_z_m=0.,
                  velocity_sigma_xy_mps=0., velocity_sigma_z_mps=0.,
                  reported_variance_xy_m2=1., reported_variance_z_m2=1.,
                  native_noise=True, adaptive_environment=False,
                  environment_range_m=80., environment_noise_gain=3.),
    'ideal': dict(position_sigma_xy_m=0., position_sigma_z_m=0.,
                 velocity_sigma_xy_mps=0., velocity_sigma_z_mps=0.,
                 reported_variance_xy_m2=.01, reported_variance_z_m2=.01,
                 native_noise=False, adaptive_environment=False,
                 environment_range_m=80., environment_noise_gain=3.),
}


def resolve(base=None, lidar=None, imu=None, gps=None, timing=None):
    p = copy.deepcopy(json.loads(Path(base or BASE).read_text()))
    models = p.setdefault('models', {})
    # Custom profiles retain their own settings unless explicitly overridden.
    if base is None:
        lidar, imu, gps, timing = lidar or 'vlp16', imu or 'mems-baseline', gps or DEFAULT_GPS, timing or 'rolling'
    if lidar:
        for key in ('pattern_file','pattern_sha256','point_rate_hz','scan_pattern'):
            p['lidar'].pop(key, None)
        p['lidar'].update(LIDARS[lidar]); models['lidar'] = lidar
    if imu:
        if imu not in IMUS:
            raise ValueError('unknown IMU preset')
        original = json.loads(BASE.read_text())
        p['imu_ros'], p['imu_px4'] = original['imu_ros'], original['imu_px4']
        if imu == 'low-noise':
            for key in p['imu_ros']:
                if any(word in key for word in ('noise_density', 'sigma', 'random_walk')):
                    p['imu_ros'][key] *= .2
            for key in p['imu_px4']:
                if not key.endswith('CorrelationTime'):
                    p['imu_px4'][key] *= .2
        models['imu'] = imu
    if gps:
        p['gps'] = copy.deepcopy(GPS[gps]); models['gps'] = gps
        p['gps_reference'] = (copy.deepcopy(GPS_REFERENCE) if gps==DEFAULT_GPS else
                              dict(id=gps,classification='uncalibrated_experimental' if gps=='standalone' else 'simulation_control',
                                   environment_model='uncalibrated_sky_fraction' if GPS[gps]['adaptive_environment'] else 'disabled'))
    p.setdefault('gps', dict(GPS['standalone']))
    p['gps'].setdefault('seed', 20261119)
    p['gps'].setdefault('bias_sigma_xy_m', 0.)
    p['gps'].setdefault('bias_sigma_z_m', 0.)
    p['gps'].setdefault('bias_tau_s', 120.)
    p['gps'].setdefault('spoof_velocity_consistent', True)
    p['gps'].setdefault('adaptive_environment', False)
    p['gps'].setdefault('environment_range_m', 80.)
    p['gps'].setdefault('environment_noise_gain', 3.)
    p['gps'].setdefault('reported_variance_mode', 'explicit_experimental')
    if p['gps']['reported_variance_mode']=='model_marginal':
        g=p['gps']
        for key in ('position_sigma_enu_m','bias_sigma_enu_m'):
            if not isinstance(g.get(key),list) or len(g[key])!=3 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<0 for v in g[key]):
                raise ValueError('GPS ENU parameters require three finite nonnegative numbers: '+key)
        variance=[a*a+b*b for a,b in zip(g['position_sigma_enu_m'],g['bias_sigma_enu_m'])]
        g['reported_variance_enu_m2']=variance
        g['reported_variance_xy_m2']=max(variance[:2]);g['reported_variance_z_m2']=variance[2]
    p['lidar'].setdefault('vertical_min_deg', -15.)
    p['lidar'].setdefault('vertical_max_deg', 15.)
    p['lidar'].setdefault('seed', 20261119)
    p['lidar'].setdefault('scan_pattern', 'uniform')
    p['lidar']['timing'] = timing or p['lidar'].get('timing', 'snapshot')
    p['schema_version'] = 2
    p['description'] = 'Resolved research sensor profile; explicit model selections and timing below.'
    p['model_note'] = ('Mid-360 official recorded scan directions replayed cyclically; acquisition quantized to physics batches, not a calibrated hardware emulator.'
                       if p['lidar']['scan_pattern']=='livox_csv' else
                       'Research parameter presets; uniform beam elevations, not vendor-calibrated firing schedules.')
    validate(p)
    reference=p.get('gps_reference',{})
    if reference.get('id')==DEFAULT_GPS:
        changed={k:p['gps'].get(k) for k,value in GPS[DEFAULT_GPS].items() if p['gps'].get(k)!=value}
        reference['parameter_overrides']=changed
        reference['classification']='user_modified_from_published_fit' if changed else GPS_REFERENCE['classification']
    return p


def validate(p):
    l, g = p['lidar'], p['gps']
    if l['timing'] not in ('snapshot', 'rolling'):
        raise ValueError('timing must be snapshot or rolling')
    for key in ('horizontal_samples', 'vertical_samples', 'seed'):
        if isinstance(l[key], bool) or not isinstance(l[key], int) or l[key] < 0:
            raise ValueError('invalid LiDAR ' + key)
    if l['horizontal_samples'] < 2 or not 2 <= l['vertical_samples'] <= 128:
        raise ValueError('invalid LiDAR beam counts')
    for key, value in l.items():
        if key in ('timing','scan_pattern','pattern_file','pattern_sha256'):
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('invalid LiDAR ' + key)
    if not -90 < l['vertical_min_deg'] < l['vertical_max_deg'] < 90:
        raise ValueError('invalid LiDAR vertical angles')
    if not 0 < l['range_min_m'] < l['range_max_m'] or l['rate_hz'] <= 0 or l['range_resolution_m'] <= 0 or l['range_noise_sigma_m'] < 0:
        raise ValueError('invalid LiDAR rate, range or noise')
    if l['scan_pattern'] not in ('uniform','livox_csv'):
        raise ValueError('unknown scan pattern')
    if l['scan_pattern']=='livox_csv':
        path=Path(l.get('pattern_file',''))
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=l.get('pattern_sha256'):
            raise ValueError('LiDAR pattern file missing or checksum mismatch')
        if l['vertical_samples']!=4 or not math.isclose(l['point_rate_hz'],l['horizontal_samples']*4*l['rate_hz']):
            raise ValueError('Mid-360 point rate must match samples per frame and frame rate')
    for key, value in g.items():
        if key=='reported_variance_mode':
            if value not in ('explicit_experimental','model_marginal'):raise ValueError('invalid GPS variance mode')
        elif key in ('position_sigma_enu_m','bias_sigma_enu_m','bias_tau_enu_s','reported_variance_enu_m2'):
            if not isinstance(value,list) or len(value)!=3 or any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or v<0 for v in value):
                raise ValueError('GPS ENU parameters require three finite nonnegative numbers: '+key)
            if key=='bias_tau_enu_s' and min(value)<=0:raise ValueError('GPS correlation times must be positive')
        elif key in ('native_noise', 'spoof_velocity_consistent', 'adaptive_environment'):
            if not isinstance(value, bool):
                raise ValueError('GPS flag must be boolean: ' + key)
        elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError('invalid GPS ' + key)
    if not isinstance(g['seed'], int) or not 0<=g['seed']<2**32 or not 0<=l['seed']<2**32 or min(g['reported_variance_xy_m2'], g['reported_variance_z_m2'], g['bias_tau_s']) <= 0:
        raise ValueError('invalid GPS seed or reported variance')
    if not 1 <= g['environment_range_m'] <= 500 or g['environment_noise_gain'] > 20:
        raise ValueError('invalid GPS environment model parameters')
    axes=('position_sigma_enu_m','bias_sigma_enu_m','bias_tau_enu_s')
    if any(k in g for k in axes) and not all(k in g for k in axes):raise ValueError('GPS ENU error parameters must be provided together')
    if g.get('reported_variance_mode')=='model_marginal':
        if not all(k in g for k in axes) or g['native_noise'] or g['adaptive_environment']:
            raise ValueError('Model-matched GPS variance requires ENU parameters, native_noise=false and adaptive_environment=false')
        expected=[a*a+b*b for a,b in zip(g['position_sigma_enu_m'],g['bias_sigma_enu_m'])]
        if 'reported_variance_enu_m2' not in g or any(not math.isclose(a,b,rel_tol=1e-10) for a,b in zip(expected,g['reported_variance_enu_m2'])):
            raise ValueError('GPS reported covariance differs from the generated position error model')
        if not math.isclose(g['reported_variance_xy_m2'],max(expected[:2])) or not math.isclose(g['reported_variance_z_m2'],expected[2]):
            raise ValueError('GPS scalar accuracy differs from the ENU covariance')
