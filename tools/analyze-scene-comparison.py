#!/usr/bin/env python3
"""Compare real Gazebo captures in three worlds under common sensor/estimator settings."""
import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import yaml
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('comparison',ROOT/'tools/compare-fastglio-spoof-runs.py')
comparison=importlib.util.module_from_spec(spec);spec.loader.exec_module(comparison)
SCENES={'simple':'简单走廊','complex_urban':'错落建筑街区','industrial_yard':'工业场地'}


def rotation(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def audit_cloud_geometry(path,origin,targets=(20,30,40,50,60)):
    """Estimate raw-cloud surface normals; this is not FAST-LIO's map matching information."""
    truth=[];chosen={};distances={};cloud_times=[]
    with rosbag.Bag(str(path)) as bag:
        for _,m,_ in bag.read_messages(topics=['/ground_truth/state']):
            truth.append((m.header.stamp.to_sec(),m))
        for _,m,_ in bag.read_messages(topics=['/velodyne_points']):
            cloud_times.append(m.header.stamp.to_sec())
            when=m.header.stamp.to_sec()-origin
            target=min(targets,key=lambda t:abs(t-when));delta=abs(target-when)
            if delta<distances.get(target,float('inf')):
                distances[target]=delta;chosen[target]=m
    times=np.array([row[0] for row in truth]);records=[]
    for target,msg in sorted(chosen.items()):
        dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,
            'offsets':[next(f.offset for f in msg.fields if f.name==key) for key in ('x','y','z')],
            'itemsize':msg.point_step})
        source=np.frombuffer(msg.data,dtype=dtype,count=msg.width*msg.height)
        points=np.column_stack([source[a] for a in ('x','y','z')]).astype(float)
        ranges=np.linalg.norm(points,axis=1)
        far_fraction=float(np.mean(ranges>95))
        points=points[np.isfinite(points).all(1)&(ranges>.5)&(ranges<60)]
        if len(points)<100:continue
        sample=points[::10]
        distance,index=cKDTree(points).query(sample,k=10)
        near=points[index];delta=near-near.mean(1,keepdims=True)
        cov=np.einsum('nki,nkj->nij',delta,delta)/10
        values,vectors=np.linalg.eigh(cov)
        spread=values[:,1]/np.maximum(values[:,2],1e-12)
        flatness=values[:,0]/np.maximum(values[:,1],1e-12)
        mask=(spread>.02)&(flatness<.2)&(distance[:,-1]<1.5)
        stamp=msg.header.stamp.to_sec();j=int(np.argmin(abs(times-stamp)))
        if abs(times[j]-stamp)>.05:raise ValueError('cloud/truth timestamp gap in geometry audit')
        pose=truth[j][1].pose.pose
        r=rotation(pose.orientation)
        normals=vectors[mask,:,0]@r.T
        axis=np.argmax(abs(normals),axis=1)
        fractions=(np.bincount(axis,minlength=3)/max(1,len(axis))).tolist()
        world_points=(points+np.array([0,0,.15]))@r.T+np.array([pose.position.x,pose.position.y,pose.position.z])
        records.append(dict(target_relative_s=target,actual_relative_s=stamp-origin,
            valid_points=len(points),near_max_range_fraction=far_fraction,
            planar_neighborhood_fraction=float(np.mean(mask)),surface_normal_axis_fraction=fractions,
            ground_height_fraction=float(np.mean(abs(world_points[:,2])<.3)),
            note='local raw-scan PCA at true orientation, range 0.5-60 m; not estimator matching normals'))
    gaps=np.diff(cloud_times)
    return dict(scan_count=len(cloud_times),
                mean_header_rate_hz=float((len(cloud_times)-1)/(cloud_times[-1]-cloud_times[0])),
                median_header_gap_ms=float(np.median(gaps)*1000),
                max_header_gap_ms=float(np.max(gaps)*1000),samples=records)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--group',type=Path,required=True)
    args=parser.parse_args();group=args.group.resolve()
    cases={};profile_hashes=set();route_hashes=set();algorithm_hashes=set();vehicle_hashes=set();world_hashes={}
    for scene in SCENES:
        for scenario in ('clean','spoofed'):
            path=group/scene/scenario
            run=comparison.read_run(path);settings=json.loads((path/'capture-settings.json').read_text())
            if not(settings['frontend']=='velodyne' and settings['point_filter_num']==4 and settings['route_clock']=='sim'):
                raise ValueError('case settings differ from benchmark design')
            profile_hashes.add(hashlib.sha256((path/'sensor_profile.json').read_bytes()).hexdigest())
            route_hashes.add(hashlib.sha256((path/'route_plan.json').read_bytes()).hexdigest())
            algorithm_hashes.add(hashlib.sha256((path/'algorithm.yaml').read_bytes()).hexdigest())
            vehicle_hashes.add(hashlib.sha256((path/'vehicle.sdf').read_bytes()).hexdigest())
            world_hashes.setdefault(scene,set()).add(hashlib.sha256((path/'world.sdf').read_bytes()).hexdigest())
            params=yaml.safe_load((path/'parameters.yaml').read_text())
            if not(params['preprocess']['lidar_type']==2 and params['point_filter_num']==4
                   and params['mapping']['bias_freeze_en'] is False):
                raise ValueError('effective frontend parameters differ')
            if not run['loose_report']['pass_live_loose_gate']:raise ValueError('missing fusion output')
            flight=run['flight_report']
            if not(flight['armed'] and flight['offboard'] and flight['max_displacement_m']>=5
                   and flight['max_climb_m']>=3 and flight['injection_match_fraction']>=.99):
                raise ValueError('basic flight or GPS injection validation failed')
            cases[(scene,scenario)]=run
    if any(len(values)!=1 for values in (profile_hashes,route_hashes,algorithm_hashes,vehicle_hashes,*world_hashes.values())):
        raise ValueError('sensor, route, algorithm, vehicle or paired world inputs differ')
    start=max(rows[0,0] for run in cases.values() for rows in run['errors'].values())
    end=min(rows[-1,0] for run in cases.values() for rows in run['errors'].values())
    results={};error_rows=[]
    for (scene,scenario),run in cases.items():
        entry=dict(metrics={},flight=run['flight_report'],fusion=run['loose_report'])
        for topic,values in run['errors'].items():
            rows=values[(values[:,0]>=start)&(values[:,0]<=end)].copy()
            rows[:,1:]-=rows[0,1:]
            entry['metrics'][topic]=comparison.metrics(rows)
            error_rows.extend((scene,scenario,topic,*row) for row in rows)
        truth=run['aligned']['truth']
        entry['truth_displacement_enu_min_m']=truth[:,1:].min(0).tolist()
        entry['truth_displacement_enu_max_m']=truth[:,1:].max(0).tolist()
        entry['surface_audit']=audit_cloud_geometry(run['path']/'sensors.bag',run['logs'][0]['sim_time_ns']/1e9)
        results.setdefault(scene,{})[scenario]=entry
    report=dict(common_relative_interval_s=[start,end],sensor_profile_sha256=next(iter(profile_hashes)),
        route_sha256=next(iter(route_hashes)),algorithm_config_sha256=next(iter(algorithm_hashes)),
        vehicle_sha256=next(iter(vehicle_hashes)),world_sha256={s:next(iter(h)) for s,h in world_hashes.items()},
        method='six independent fresh PX4 flights; sim-time waypoint dwell; '
        'common time window, first matched point translation; no rotation/scale fit',
        limitation='one flight per world/condition; not a statistical causal proof; GNSS source is truth plus scripted bias, '
        'not geometry-driven multipath',scenes=results)
    (group/'scene-results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    with (group/'scene-errors.csv').open('w',newline='') as file:
        writer=csv.writer(file);writer.writerow(('scene','scenario','series','relative_time_s','east_error_m','north_error_m','up_error_m'))
        writer.writerows(error_rows)
    plt.rcParams.update({'font.family':'Noto Sans CJK SC','font.size':11})
    fig,axes=plt.subplots(1,2,figsize=(13,5.5));x=np.arange(3)
    for ax,topic,title in zip(axes,('/Odometry','/loose_odom'),('纯 LiDAR/IMU','LiDAR＋IMU＋GPS 松耦合')):
        for i,(scenario,label,color) in enumerate([('clean','无欺骗','#168aad'),('spoofed','有 GPS 欺骗','#d6336c')]):
            vals=[results[scene][scenario]['metrics'][topic]['rmse_3d_m'] for scene in SCENES]
            bars=ax.bar(x+(i-.5)*.34,vals,width=.32,color=color,label=label)
            ax.bar_label(bars,fmt='%.3f',padding=3,fontsize=10)
        ax.set_xticks(x,list(SCENES.values()));ax.set_ylabel('三维 RMSE / m');ax.set_title(title)
        ax.legend();ax.margins(y=.4);ax.grid(axis='y',alpha=.2)
    fig.suptitle('相同传感器与算法参数：场景复杂度对照',fontsize=17)
    fig.tight_layout(rect=(0,.02,1,.94));fig.savefig(group/'scene-rmse-comparison.png',dpi=180)
    fig.savefig(group/'scene-rmse-comparison.pdf');plt.close(fig)
    fig,axes=plt.subplots(3,3,figsize=(16,11),sharex=True,sharey='row')
    for col,(scene,label) in enumerate(SCENES.items()):
        for row,axis_name in enumerate(('东 E','北 N','天 U')):
            ax=axes[row,col]
            for scenario,color in [('clean','#168aad'),('spoofed','#d6336c')]:
                for topic,style,method in [('/Odometry','--','纯 LIO'),('/loose_odom','-','松耦合')]:
                    a=cases[(scene,scenario)]['errors'][topic]
                    mask=(a[:,0]>=start)&(a[:,0]<=end);a=a[mask].copy();a[:,1:]-=a[0,1:]
                    ax.plot(a[:,0],a[:,row+1],color=color,linestyle=style,linewidth=1.2,
                            label=('无欺骗' if scenario=='clean' else '有欺骗')+' '+method)
            ax.axvline(20,color='#888',linestyle=':',linewidth=1)
            ax.grid(alpha=.2);ax.set_xlim(start,end)
            if col==0:ax.set_ylabel(axis_name+'误差 / m')
            if row==0:ax.set_title(label,fontsize=15)
            if row==2:ax.set_xlabel('相对 GPS 源启动的仿真时间 / s')
    axes[0,0].legend(fontsize=8,ncol=2)
    fig.suptitle('三种场景的 ENU 时序误差；虚线为纯 LIO，实线为松耦合',fontsize=17)
    fig.tight_layout(rect=(0,0,1,.95));fig.savefig(group/'scene-enu-comparison.png',dpi=180)
    fig.savefig(group/'scene-enu-comparison.pdf');plt.close(fig)
    print(json.dumps({scene:{scenario:{topic:round(m['rmse_3d_m'],3) for topic,m in case['metrics'].items()}
                           for scenario,case in rows.items()} for scene,rows in results.items()},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
