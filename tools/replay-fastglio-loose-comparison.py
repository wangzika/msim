#!/usr/bin/env python3
"""Recompute pure LIO and GNSS loose coupling with the same improved frontend."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import rosbag
import yaml

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('annotator',ROOT/'tools/annotate-gazebo-lidar.py')
annotator=importlib.util.module_from_spec(spec)
spec.loader.exec_module(annotator)

INPUT_TOPICS=['/imu/data','/ground_truth/state','/msim/spoofed_gnss_odom']
OUTPUT_TOPICS=['/Odometry','/path','/loose_odom','/loose_path','/rtk_path','/gnss_loose/diagnostics']


def stop(process):
    if process is None or process.poll() is not None:return
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
        try:
            os.killpg(process.pid,sig)
            process.wait(timeout=8)
            return
        except subprocess.TimeoutExpired:pass
        except ProcessLookupError:return


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--port',type=int,default=11340)
    parser.add_argument('--rate',type=float,default=.5)
    parser.add_argument('--stride',type=int,default=2)
    args=parser.parse_args()
    source=args.source.resolve();out=args.output.resolve()
    if args.rate<=0 or args.stride<1:parser.error('rate and stride must be positive')
    out.mkdir(parents=True,exist_ok=False)
    for name in ('sensor_profile.json','route_plan.json','injected-gps.jsonl','px4-flight-report.json'):
        shutil.copy2(source/name,out/name)
    config=yaml.safe_load((ROOT/'adapters/ros1_noetic/fast_glio_uav/px4_replay.yaml').read_text())
    config['common']['lid_topic']=annotator.OUTPUT_TOPIC
    config['preprocess']['lidar_type']=2
    config['mapping']['bias_freeze_en']=False
    (out/'algorithm.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    launch=out/'algorithm.launch'
    shutil.copy2(ROOT/'adapters/ros1_noetic/fast_glio_uav/px4_live_loose.launch',launch)
    input_bag=out/'input.bag'
    counts={topic:0 for topic in [*INPUT_TOPICS,annotator.OUTPUT_TOPIC]}
    print('Preparing sensor and GNSS input: '+str(out),flush=True)
    with rosbag.Bag(str(source/'sensors.bag')) as original,rosbag.Bag(str(input_bag),'w',compression='lz4') as target:
        for topic,msg,stamp in original.read_messages(topics=[*INPUT_TOPICS,'/velodyne_points']):
            if topic=='/velodyne_points':
                msg=annotator.annotate(msg);topic=annotator.OUTPUT_TOPIC
            target.write(topic,msg,stamp);counts[topic]+=1
    if not all(counts.values()):raise ValueError('source input is missing required topics')
    manifest=dict(mode='offline_recompute_of_existing_closed_loop_flight',source_run=str(source),
                  source_bag_sha256=hashlib.sha256((source/'sensors.bag').read_bytes()).hexdigest(),
                  source_topic_counts=counts,point_filter_num=args.stride,lidar_type=2,
                  point_time='uniform 1 us instantaneous Gazebo ray snapshot, not spinning scan timing',
                  rate=args.rate,gnss_feedback_to_lio=False,
                  inherited_flight_validation=True,
                  inherited_validation_note='PX4 flight and online LiDAR/GPS pairing report copied from original capture; no new PX4 flight was performed')
    (out/'replay-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    env=dict(os.environ,ROS_MASTER_URI=f'http://127.0.0.1:{args.port}',ROS_IP='127.0.0.1',
             ROS_HOME=str(out/'ros-home'),ROS_LOG_DIR=str(out/'roslogs'))
    Path(env['ROS_HOME']).mkdir();Path(env['ROS_LOG_DIR']).mkdir()
    master=mapper=recorder=None
    logs={name:(out/(name+'.log')).open('w') for name in ('roscore','algorithm','record','play')}
    try:
        master=subprocess.Popen(['roscore','-p',str(args.port)],env=env,stdout=logs['roscore'],stderr=subprocess.STDOUT,start_new_session=True)
        import rosgraph
        ros_master=rosgraph.Master('/msim_replay_check',master_uri=env['ROS_MASTER_URI'])
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            if master.poll() is not None:raise RuntimeError('ROS master exited')
            try:ros_master.getPid();break
            except Exception:time.sleep(.1)
        else:raise RuntimeError('ROS master unavailable')
        mapper=subprocess.Popen(['roslaunch',str(launch),'config_file:='+str(out/'algorithm.yaml'),
            'rviz_config:='+str(ROOT/'adapters/ros1_noetic/fast_glio_uav/px4_live_loose.rviz'),
            'rviz:=false','point_filter_num:='+str(args.stride)],env=env,stdout=logs['algorithm'],stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+40
        while time.monotonic()<deadline:
            if mapper.poll() is not None:raise RuntimeError('mapping launch exited')
            state=ros_master.getSystemState()
            subscribed={topic for topic,nodes in state[1] if '/laserMapping' in nodes}
            if {annotator.OUTPUT_TOPIC,'/imu/data','/msim/spoofed_gnss_odom'}<=subscribed:break
            time.sleep(.1)
        else:raise RuntimeError('mapper did not subscribe to all three sensor streams')
        subprocess.run(['rosparam','dump',str(out/'parameters.yaml')],env=env,check=True)
        recorder=subprocess.Popen(['rosbag','record','--lz4','-O',str(out/'sensors.bag'),
            '/clock',*INPUT_TOPICS,annotator.OUTPUT_TOPIC,*OUTPUT_TOPICS],env=env,
            stdout=logs['record'],stderr=subprocess.STDOUT,start_new_session=True)
        time.sleep(1.5)
        print('Replaying LiDAR + IMU + GPS: '+str(out),flush=True)
        player=subprocess.Popen(['rosbag','play','-q','--clock','-r',str(args.rate),str(input_bag)],env=env,
            stdout=logs['play'],stderr=subprocess.STDOUT,start_new_session=True)
        while player.poll() is None:
            if mapper.poll() is not None or recorder.poll() is not None:
                stop(player);raise RuntimeError('mapping or recorder exited during replay')
            time.sleep(.5)
        if player.returncode:raise RuntimeError('rosbag playback failed')
        time.sleep(2)
        stop(recorder);recorder=None
    finally:
        stop(recorder);stop(mapper);stop(master)
        for file in logs.values():file.close()
    with rosbag.Bag(str(out/'sensors.bag')) as bag:
        info=bag.get_type_and_topic_info().topics
        actual={topic:info[topic].message_count if topic in info else 0 for topic in [*INPUT_TOPICS,annotator.OUTPUT_TOPIC,*OUTPUT_TOPICS]}
        diagnostics=[msg.data for _,msg,_ in bag.read_messages(topics=['/gnss_loose/diagnostics'])]
    accepted=sum(len(row)>2 and int(row[1])==4 and row[2]==1.0 for row in diagnostics)
    report=dict(mode='offline_replay',topic_counts=actual,gnss_alignment_reached=any(len(row)>2 and row[2]==1 for row in diagnostics),
                gnss_updates_accepted=accepted,pass_live_loose_gate=bool(all(actual.values()) and accepted>=5))
    (out/'fastglio-live-report.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['pass_live_loose_gate']:raise RuntimeError('loose coupling output validation failed')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
