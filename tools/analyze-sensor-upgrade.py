#!/usr/bin/env python3
"""Check moving scans against known static geometry and summarize capture gates."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import rosbag


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);a=ap.parse_args()
    truth=[]
    with rosbag.Bag(str(a.run/'sensors.bag')) as bag:
        for _,m,_ in bag.read_messages(topics=['/ground_truth/state']):
            p,q=m.pose.pose.position,m.pose.pose.orientation
            truth.append([m.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    truth=np.array(truth);ts=truth[:,0]
    boxes=[]
    world=ET.parse(a.run/'world.sdf').getroot().find('world')
    for model in world.findall('model'):
        pose=np.fromstring(model.findtext('pose','0 0 0 0 0 0'),sep=' ')
        for link in model.findall('link'):
            lp=np.fromstring(link.findtext('pose','0 0 0 0 0 0'),sep=' ')
            for collision in link.findall('collision'):
                text=collision.findtext('geometry/box/size')
                if text is None:continue
                cp=np.fromstring(collision.findtext('pose','0 0 0 0 0 0'),sep=' ')
                if np.linalg.norm(pose[3:])+np.linalg.norm(lp[3:])+np.linalg.norm(cp[3:])>1e-8:
                    raise ValueError('This geometry check requires axis-aligned boxes')
                boxes.append((pose[:3]+lp[:3]+cp[:3],np.fromstring(text,sep=' ')/2))

    def transform(points,times):
        i=np.clip(np.searchsorted(ts,times)-1,0,len(ts)-2)
        alpha=((times-ts[i])/(ts[i+1]-ts[i]))[:,None]
        p=truth[i,1:4]*(1-alpha)+truth[i+1,1:4]*alpha
        q0,q1=truth[i,4:8],truth[i+1,4:8].copy()
        q1[np.sum(q0*q1,axis=1)<0]*=-1
        q=q0*(1-alpha)+q1*alpha;q/=np.linalg.norm(q,axis=1)[:,None]
        v=points+np.array([0,0,.15])
        return v+2*np.cross(q[:,:3],np.cross(q[:,:3],v)+q[:,3,None]*v)+p

    def residual(p):
        distance=abs(p[:,2])
        for center,half in boxes:
            q=abs(p-center)-half
            d=abs(np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0))
            distance=np.minimum(distance,d)
        return distance

    correct=[];begin=[];end=[];frames=0
    with rosbag.Bag(str(a.run/'sensors.bag')) as bag:
        for frame,(_,m,_) in enumerate(bag.read_messages(topics=['/velodyne_points'])):
            if frame%5 or m.width==0:continue
            t=m.header.stamp.to_sec();i=np.clip(np.searchsorted(ts,t),1,len(ts)-1)
            speed=np.linalg.norm(truth[i,1:4]-truth[i-1,1:4])/(ts[i]-ts[i-1])
            if speed<.5:continue
            dtype=np.dtype({'names':['x','y','z','time'],'formats':['<f4']*4,'offsets':[0,4,8,24],'itemsize':m.point_step})
            data=np.frombuffer(m.data,dtype=dtype)[::17]
            xyz=np.column_stack([data[k] for k in ('x','y','z')])
            keep=np.linalg.norm(xyz,axis=1)>1
            xyz=xyz[keep];offset=data['time'][keep].astype(float)
            if not len(xyz):continue
            correct.extend(residual(transform(xyz,t+offset)))
            begin.extend(residual(transform(xyz,np.full(len(xyz),t))))
            end.extend(residual(transform(xyz,np.full(len(xyz),t+float(data['time'].max())))))
            frames+=1
    stats=lambda values:dict(rmse_m=float(np.sqrt(np.mean(np.square(values)))),p95_m=float(np.quantile(values,.95)))
    rows=[json.loads(s) for s in (a.run/'injected-gps.jsonl').read_text().splitlines()]
    report=dict(moving_frames=frames,moving_points=len(correct),
                actual_point_time_surface_error=stats(correct),
                all_points_at_frame_start_surface_error=stats(begin),
                all_points_at_frame_end_surface_error=stats(end),
                attack_pair_max_error_m=max(abs(r['gps_east_m']-r['normal_east_m']-r['attack_east_m']) for r in rows),
                gps_actual_rate_hz=(len(rows)-1)/((rows[-1]['sim_time_ns']-rows[0]['sim_time_ns'])/1e9),
                gps_publish_delay_ms=np.quantile([(r['publish_time_ns']-r['sim_time_ns'])/1e6 for r in rows],[0,.5,1]).tolist())
    # RMS of errors, rather than magnitude of the mean error.
    e=np.array([[r['normal_east_m']-r['east_m'],r['normal_north_m']-r['north_m'],r['normal_up_m']-r['up_m']] for r in rows])
    report['normal_gps_enu_absolute_rmse_m']=np.sqrt(np.mean(e*e,axis=0)).tolist()
    report['pass_moving_scan_gate']=bool(frames>10 and stats(correct)['rmse_m']<.025 and stats(correct)['rmse_m']<min(stats(begin)['rmse_m'],stats(end)['rmse_m']))
    for filename in ('px4-flight-report.json','fastglio-live-report.json'):
        report[filename]=json.loads((a.run/filename).read_text())
    (a.run/'sensor-upgrade-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['pass_moving_scan_gate']:raise SystemExit('moving scan check failed')


if __name__=='__main__':main()
