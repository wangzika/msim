#!/usr/bin/env python3
"""Integration checks in a closed room with analytic ray intersections.

Run inside the configured Gazebo/ROS environment, without any PX4 process.
"""
import argparse
import copy
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rosgraph
import rospy
from sensor_msgs.msg import PointCloud2
from livox_ros_driver.msg import CustomMsg
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'adapters/ros1_noetic/msim_sensors'))
from sensor_profiles import resolve
spec = importlib.util.spec_from_file_location('prepare', ROOT/'adapters/ros1_noetic/fast_glio_uav/prepare_px4_assets.py')
prepare = importlib.util.module_from_spec(spec); spec.loader.exec_module(prepare)


def stop(p):
    if p and p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL); p.wait()


def fixture(out, profile):
    source = ROOT/'build/deps/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf'
    model_file = out/'vehicle.sdf'
    prepare.prepare_model(source, model_file, online_gnss=True, sensor_profile=profile)
    vehicle = ET.parse(model_file).getroot().find('model')
    root = ET.Element('sdf', version='1.6'); world = ET.SubElement(root, 'world', name='default')
    prepare.add_xml(world, '<physics name="default" type="ode"><max_step_size>0.002</max_step_size><real_time_update_rate>500</real_time_update_rate></physics>')
    model = ET.SubElement(world, 'model', name='iris')
    initial=ET.fromstring('<parts><static>true</static><pose>0 0 1 0 0 0</pose><link name="base_link"><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial></link></parts>')
    model.extend(list(initial))
    for query in ("model[@name='gps0']", "joint[@name='gps0_joint']", "plugin[@name='msim_gps_relay']", "plugin[@name='msim_scanning_lidar']"):
        model.append(copy.deepcopy(vehicle.find(query)))
    for name, pos, size in [('east',(5.1,0,2),(.2,10.4,4)), ('west',(-5.1,0,2),(.2,10.4,4)),
                             ('north',(0,5.1,2),(10,.2,4)), ('south',(0,-5.1,2),(10,.2,4)),
                             ('floor',(0,0,-.1),(10,10,.2)), ('ceiling',(0,0,4.1),(10,10,.2))]:
        prepare.add_xml(world, f'<model name="{name}"><static>true</static><pose>{" ".join(map(str,pos))} 0 0 0</pose><link name="body"><collision name="box"><geometry><box><size>{" ".join(map(str,size))}</size></box></geometry></collision></link></model>')
    ET.ElementTree(root).write(out/'room.world', encoding='unicode', xml_declaration=True)


def cloud_array(msg):
    dtype = np.dtype({'names':['x','y','z','ring','time'], 'formats':['<f4','<f4','<f4','<u2','<f4'],
                      'offsets':[0,4,8,20,24], 'itemsize':32})
    return np.frombuffer(msg.data, dtype=dtype)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args(); a.output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11401'
    os.environ['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:11402'
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''
    os.environ['MSIM_SPOOF_START_S'] = '0';os.environ['MSIM_SPOOF_EAST_RATE_MPS'] = '.2'
    master = subprocess.Popen(['roscore','-p','11401'], stdout=(a.output/'roscore.log').open('w'), stderr=subprocess.STDOUT, start_new_session=True)
    server = None
    results=[]
    try:
        for _ in range(100):
            try:
                rosgraph.Master('/check').getPid();break
            except Exception:
                time.sleep(.1)
        rospy.init_node('msim_sensor_validation', disable_signals=True)
        cases=[('vlp16','snapshot'),('vlp16','rolling'),('hdl32','rolling'),('os1-64','rolling'),('mid360','snapshot'),('mid360','rolling')]
        for name, timing in cases:
            out=a.output/(name+'-'+timing); out.mkdir(exist_ok=True)
            profile=resolve(lidar=name, timing=timing)
            (out/'sensor_profile.json').write_text(json.dumps(profile,indent=2)+'\n')
            fixture(out,profile)
            clouds=[];samples=[];livox=[]
            subs=[rospy.Subscriber('/velodyne_points',PointCloud2,lambda m:clouds.append(m),queue_size=30),
                  rospy.Subscriber('/msim/gps_samples',String,lambda m:samples.append(json.loads(m.data)),queue_size=50)]
            if name=='mid360':
                subs.append(rospy.Subscriber('/livox/lidar',CustomMsg,lambda m:livox.append(m),queue_size=30,buff_size=8*1024*1024))
            server=subprocess.Popen(['rosrun','gazebo_ros','gzserver','--verbose',str(out/'room.world')],
                                    stdout=(out/'gazebo.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+40
            while (len(clouds)<16 or len(samples)<8) and time.monotonic()<deadline:
                if server.poll() is not None:raise RuntimeError('Gazebo exited: '+str(out))
                time.sleep(.1)
            assert len(clouds)>=16 and len(samples)>=8, 'sensor output timeout: '+str(out)
            # Exercise online command input, independent of CLI-scheduled attack.
            command=rospy.Publisher('/msim/gps_attack',String,queue_size=1)
            deadline=time.monotonic()+10
            while command.get_num_connections()==0 and time.monotonic()<deadline:time.sleep(.05)
            command.publish(String(data=json.dumps(dict(enabled=True,east_offset_m=3.,velocity_consistent=True))))
            deadline=time.monotonic()+10
            while not any(abs(s['attack_east_m']-3.)<1e-8 for s in samples) and time.monotonic()<deadline:time.sleep(.05)
            assert any(abs(s['attack_east_m']-3.)<1e-8 for s in samples), 'online command did not apply'
            for sub in subs:sub.unregister()
            command.unregister();stop(server);server=None
            arrays=[cloud_array(c) for c in clouds[:16]]
            data=np.concatenate(arrays); xyz=np.column_stack([data[k] for k in ('x','y','z')])
            ranges=np.linalg.norm(xyz,axis=1); dirs=xyz/ranges[:,None]
            horizontal=np.minimum(5/np.maximum(abs(dirs[:,0]),1e-12),5/np.maximum(abs(dirs[:,1]),1e-12))
            vertical=np.where(dirs[:,2]>0,2.85,1.15)/np.maximum(abs(dirs[:,2]),1e-12)
            residual=ranges-np.minimum(horizontal,vertical)
            spans=[float(np.ptp(c['time'])) for c in arrays]
            row=dict(lidar=name,timing=timing,cloud_count=len(clouds),gps_count=len(samples),
                     points_per_frame=[min(len(c) for c in arrays),max(len(c) for c in arrays)],
                     ring_count=len(np.unique(data['ring'])),range_noise_mean_m=float(residual.mean()),
                     range_noise_std_m=float(residual.std()),point_time_span_s=[min(spans),max(spans)],
                     frame_interval_s=float(np.median(np.diff([c.header.stamp.to_sec() for c in clouds[:16]]))),
                     gps_delay_ms=[(s['publish_time_ns']-s['sim_time_ns'])/1e6 for s in samples],
                     attack_pair_error_m=max(abs((s['gps_east_m']-s['normal_east_m'])-s['attack_east_m']) for s in samples),
                     gps_normal_east_range_m=float(np.ptp([s['normal_east_m'] for s in samples])),
                     online_command_passed=True)
            assert row['ring_count']==profile['lidar']['vertical_samples']
            assert row['points_per_frame']==[profile['lidar']['horizontal_samples']*profile['lidar']['vertical_samples']]*2
            assert abs(row['range_noise_mean_m'])<.001 and .019<row['range_noise_std_m']<.021, row
            assert abs(row['frame_interval_s']-.1)<1e-5
            assert row['attack_pair_error_m']<1e-8 and row['gps_normal_east_range_m']>.01
            assert (min(spans)>.09 if timing=='rolling' else max(spans)==0)
            if name=='mid360':
                by_seq={m.header.seq:m for m in livox}
                checked=0
                for msg in clouds[:16]:
                    if msg.header.seq not in by_seq:continue
                    native=by_seq[msg.header.seq];c=cloud_array(msg)
                    assert native.header.stamp==msg.header.stamp and native.point_num==msg.width
                    assert native.timebase==msg.header.stamp.to_nsec()
                    native_xyz=np.array([(p.x,p.y,p.z) for p in native.points])
                    assert np.array_equal(native_xyz,np.column_stack([c[k] for k in ('x','y','z')]))
                    assert all(p.line==r and p.tag==0x10 for p,r in zip(native.points,c['ring']))
                    times=np.array([p.offset_time for p in native.points])*1e-9
                    assert np.max(abs(times-c['time']))<1e-8 and np.all(np.diff(times)>=0)
                    checked+=1
                assert checked>=10,'native Livox messages missing'
                # Radial noise cannot change direction. Different frames must
                # advance along the CSV rather than repeat a fixed grid.
                ds=[]
                for c in arrays[:3]:
                    xyz=np.column_stack([c[k] for k in ('x','y','z')]);ds.append(xyz/np.linalg.norm(xyz,axis=1)[:,None])
                assert not np.allclose(ds[0],ds[1],atol=1e-5)
                row.update(native_livox_pairs_checked=checked,recorded_pattern_advances=True,
                           observed_elevation_deg=[float(np.degrees(np.arcsin(dirs[:,2])).min()),float(np.degrees(np.arcsin(dirs[:,2])).max())])
            results.append(row);print(json.dumps(row),flush=True)
            (a.output/'validation.json').write_text(json.dumps(dict(pass_all=len(results)==len(cases),results=results),indent=2)+'\n')
    finally:
        stop(server);stop(master)


if __name__=='__main__':main()
