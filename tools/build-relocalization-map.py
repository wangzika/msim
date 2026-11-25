#!/usr/bin/env python3
"""Build a sensor-only prior map bundle from a completed LiDAR/IMU flight."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'adapters/ros1_noetic/fast_glio_uav'))
import numpy as np
from autonomy_core import descriptor,voxel
from lio_map_recorder import cloud_points


def main():
    import rosbag
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();source=args.run.resolve();out=args.output.resolve()
    if out.exists():raise ValueError('Refusing to overwrite an existing map bundle')
    positions=[];stamps=[]
    with rosbag.Bag(str(source/'sensors.bag')) as bag:
        for _,msg,_ in bag.read_messages(topics=['/Odometry']):
            p=msg.pose.pose.position;positions.append([p.x,p.y,p.z]);stamps.append(msg.header.stamp.to_sec())
    if len(positions)<10:raise ValueError('No usable LiDAR/IMU trajectory')
    positions=np.asarray(positions);stamps=np.asarray(stamps);clouds=[];keys=[];descriptors=[];last=None
    with rosbag.Bag(str(source/'sensors.bag')) as bag:
        for _,msg,_ in bag.read_messages(topics=['/cloud_registered']):
            if msg.header.frame_id!='camera_init':raise ValueError('Unexpected registered cloud frame')
            i=int(np.argmin(abs(stamps-msg.header.stamp.to_sec())))
            if abs(stamps[i]-msg.header.stamp.to_sec())>.06:continue
            p=positions[i]
            if last is not None and np.linalg.norm(p-last)<1.5:continue
            points=voxel(cloud_points(msg)[:,:3],size=.5,maximum=12000)
            if len(points)<100:continue
            clouds.append(points);keys.append(p);descriptors.append(descriptor(points-p));last=p
    if not clouds:raise ValueError('No registered clouds; build map with --teleop or --autonomy')
    points=voxel(np.concatenate(clouds),maximum=120000);out.mkdir(parents=True)
    np.savez_compressed(out/'map.npz',points=points.astype(np.float32),positions=np.asarray(keys),descriptors=np.asarray(descriptors))
    info=dict(schema_version=1,frame='camera_init',home_position=positions[0].tolist(),source_run=str(source),
              scene=json.loads((source/'capture-settings.json').read_text()).get('scene_key'),keyframes=len(keys),points=len(points),
              source_topics=['/Odometry','/cloud_registered'],truth_used=False,
              algorithm='height-span ring descriptor and multistart point-to-point ICP; independent baseline, not Scan Context++ or GICP',
              sha256=hashlib.sha256((out/'map.npz').read_bytes()).hexdigest())
    (out/'map.json').write_text(json.dumps(info,indent=2)+'\n');print(json.dumps(info,indent=2))


if __name__=='__main__':main()
