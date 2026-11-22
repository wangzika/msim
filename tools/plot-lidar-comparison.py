#!/usr/bin/env python3
"""Compare current LiDAR presets with interpolated truth and common coverage."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np
import rosbag
import yaml

MODELS = [('vlp16','VLP-16','#2677B5','-'),('hdl32','HDL-32E','#E88A26','--'),
          ('os1-64','OS1-64','#32936F','-.'),('mid360','Mid-360','#C94C76',':')]
SOLUTIONS = [('/Odometry','LiDAR/IMU','lidar-imu'),
             ('/loose_odom','LiDAR/IMU/GPS 松组合','loose')]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_run(path):
    truth=[];tracks={topic:[] for topic,_,_ in SOLUTIONS}
    with rosbag.Bag(str(path/'sensors.bag')) as bag:
        for topic,msg,_ in bag.read_messages(topics=['/ground_truth/state',*tracks]):
            p=msg.pose.pose.position
            row=[msg.header.stamp.to_sec(),p.x,p.y,p.z]
            if not all(math.isfinite(v) for v in row):
                raise ValueError('Non-finite pose: '+str(path))
            (truth if topic=='/ground_truth/state' else tracks[topic]).append(row)
    truth=np.array(sorted(truth));times=truth[:,0]
    if len(times)<20 or np.any(np.diff(times)<=0):
        raise ValueError('Truth timestamps not strictly increasing: '+str(path))
    logs=[json.loads(line) for line in (path/'injected-gps.jsonl').read_text().splitlines()]
    if not logs or any(row.get('attack_active') or abs(row.get('attack_east_m',0))+abs(row.get('attack_north_m',0))>1e-9 for row in logs):
        raise ValueError('Expected a clean flight: '+str(path))
    origin=logs[0]['gnss_origin_sim_time_ns']/1e9
    errors={}
    for topic,track in tracks.items():
        data=np.array(sorted(track))
        if len(data)<20:raise ValueError('Too few odometry poses: '+str(path))
        data=data[(data[:,0]>=times[0])&(data[:,0]<=times[-1])]
        i=np.searchsorted(times,data[:,0]);i=np.clip(i,1,len(times)-1)
        valid=(times[i]-times[i-1])<=.05
        data=data[valid]
        reference=np.column_stack([np.interp(data[:,0],times,truth[:,axis]) for axis in (1,2,3)])
        error=data[:,1:]-reference
        error-=error[0]  # Translation only, before selecting the common window.
        errors[topic]=np.column_stack([data[:,0]-origin,error])
    return dict(path=path,errors=errors,origin_sim_s=origin,
                profile=json.loads((path/'sensor_profile.json').read_text()),
                settings=json.loads((path/'capture-settings.json').read_text()),
                algorithm=yaml.safe_load((path/'algorithm.yaml').read_text()),
                flight=json.loads((path/'px4-flight-report.json').read_text()),
                fusion=json.loads((path/'fastglio-live-report.json').read_text()))


def validate_inputs(runs):
    baseline=next(iter(runs.values()))
    for name,run in runs.items():
        for section in ('imu_ros','imu_px4','gps'):
            if run['profile'][section]!=baseline['profile'][section]:
                raise ValueError('Different '+section+' parameters: '+name)
        for field in ('duration_s','route_clock','flight_pattern','point_filter_num','fusion_processing_delay_s','gnss_source'):
            if run['settings'][field]!=baseline['settings'][field]:
                raise ValueError('Different capture setting '+field+': '+name)
        for field in ('timing','rate_hz','range_resolution_m','range_noise_sigma_m','seed'):
            if run['profile']['lidar'][field]!=baseline['profile']['lidar'][field]:
                raise ValueError('Different common LiDAR setting '+field+': '+name)
        for field in ('mapping','gnss_loose'):
            if run['algorithm'][field]!=baseline['algorithm'][field]:
                raise ValueError('Different algorithm '+field+' configuration: '+name)
        for filename in ('world.sdf','route_plan.json','sensor-plugins.json'):
            if sha(run['path']/filename)!=sha(baseline['path']/filename):
                raise ValueError('Different '+filename+': '+name)
        if run['settings']['point_filter_num']!=1:
            raise ValueError('Benchmark must use all points')


def metrics(rows):
    e=rows[:,1:];norm=np.linalg.norm(e,axis=1);axis=np.sqrt(np.mean(e*e,axis=0))
    value=dict(samples=len(e),start_sim_s=float(rows[0,0]),end_sim_s=float(rows[-1,0]),
               rmse_enu_m=axis.tolist(),horizontal_rmse_m=float(np.sqrt(np.mean(np.sum(e[:,:2]**2,axis=1)))),
               rmse_3d_m=float(np.sqrt(np.mean(norm*norm))),mae_enu_m=np.mean(abs(e),axis=0).tolist(),
               p95_3d_error_m=float(np.quantile(norm,.95)),max_3d_error_m=float(norm.max()),
               final_enu_error_m=e[-1].tolist(),final_3d_error_m=float(norm[-1]))
    assert math.isclose(value['rmse_3d_m']**2,float(np.sum(axis**2)),rel_tol=1e-10,abs_tol=1e-12)
    assert np.all(np.array(value['mae_enu_m'])<=axis+1e-12)
    return value


def plot(runs,topic,title,filename,output,window,yscale):
    fig,axes=plt.subplots(3,1,figsize=(12,9.8),sharex=True,facecolor='white')
    for axis,label in enumerate(('东 E','北 N','天 U')):
        ax=axes[axis];ax.axhline(0,color='#8F969E',linewidth=.8)
        for name,display,color,style in MODELS:
            rows=runs[name]['errors'][topic]
            # Break long missing-output intervals; do not interpolate estimates.
            cut=np.where(np.diff(rows[:,0])>1)[0]+1
            for j,chunk in enumerate(np.split(rows,cut)):
                ax.plot(chunk[:,0],chunk[:,axis+1],label=display if j==0 else None,
                        color=color,linestyle=style,linewidth=2.1)
        ax.set_ylabel(label+' 误差 / m',fontsize=21,labelpad=12)
        ax.grid(color='#E2E6EA',linewidth=.8);ax.tick_params(labelsize=17)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
        if yscale=='symlog':ax.set_yscale('symlog',linthresh=.2,linscale=.7,base=10)
        ax.spines[['top','right']].set_visible(False)
    axes[-1].set_xlabel('仿真时间 / s',fontsize=21,labelpad=10)
    axes[-1].set_xlim(*window)
    handles,labels=axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.5,.96),ncol=4,
               frameon=False,fontsize=18,handlelength=2.5,columnspacing=1.5)
    fig.suptitle('无欺骗 · '+title,fontsize=23,y=.995)
    fig.subplots_adjust(left=.14,right=.98,bottom=.1,top=.88,hspace=.2)
    if yscale=='symlog':
        fig.text(.5,.012,'纵轴采用对称对数刻度，±0.2 m 内为线性',ha='center',fontsize=13,color='#59636D')
    fig.savefig(output/filename,dpi=240,facecolor='white');plt.close(fig)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--group',type=Path,required=True)
    ap.add_argument('--yscale',choices=('linear','symlog','both'),default='both')
    a=ap.parse_args();group=a.group.resolve();output=group/'figures';output.mkdir(exist_ok=True)
    runs={name:read_run(group/name) for name,*_ in MODELS};validate_inputs(runs)
    rows=[v for r in runs.values() for v in r['errors'].values()]
    window=[math.ceil(max(x[0,0] for x in rows)*10)/10,
            math.floor(min(x[-1,0] for x in rows)*10)/10]
    if window[1]-window[0]<30:raise ValueError('Less than 30 s common coverage')
    for run in runs.values():
        run['errors']={topic:data[(data[:,0]>=window[0])&(data[:,0]<=window[1])] for topic,data in run['errors'].items()}
    plt.rcParams.update({'font.family':'Noto Sans CJK SC','axes.unicode_minus':False,'font.size':18})
    scales=('linear','symlog') if a.yscale=='both' else (a.yscale,)
    for topic,label,stem in SOLUTIONS:
        for scale in scales:
            filename=stem+'-enu'+('-symlog' if scale=='symlog' else '')+'.png'
            plot(runs,topic,label,filename,output,window,scale)
    summary=dict(common_sim_interval_s=window,
                 method='truth position interpolated at measurement timestamp; first matched translation only; no rotation/scale fit; raw errors, no smoothing',
                 time_origin='first native GNSS sample in each flight',
                 scenario='clean, separate closed-loop flights with the same route',
                 caveat='preset comparison: geometry, range, point count and input format differ; not a vendor hardware ranking or isolated beam-count experiment',runs={})
    with (output/'enu-errors.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['lidar','solution','time_sim_s','east_error_m','north_error_m','up_error_m'])
        for name,run in runs.items():
            entry=dict(source=str(run['path']),profile=run['profile'],settings=run['settings'],
                       input_sha256={fn:sha(run['path']/fn) for fn in ('world.sdf','route_plan.json','sensor_profile.json','algorithm.yaml','sensor-plugins.json')},
                       flight_gate=run['flight'],fusion_gate=run['fusion'],metrics={})
            for topic,_,stem in SOLUTIONS:
                entry['metrics'][stem]=metrics(run['errors'][topic])
                writer.writerows((name,stem,*row) for row in run['errors'][topic])
            summary['runs'][name]=entry
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    with (output/'metrics.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['lidar','solution','samples','east_rmse_m','north_rmse_m','up_rmse_m','horizontal_rmse_m','three_d_rmse_m','p95_3d_m','max_3d_m','final_3d_m'])
        for name,entry in summary['runs'].items():
            for stem,m in entry['metrics'].items():
                writer.writerow([name,stem,m['samples'],*m['rmse_enu_m'],m['horizontal_rmse_m'],m['rmse_3d_m'],m['p95_3d_error_m'],m['max_3d_error_m'],m['final_3d_error_m']])
    print(output)


if __name__=='__main__':main()
