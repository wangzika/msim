"""Optional ROS1 import using rosbags, never importing ROS runtime packages.

Mapping is specific to msim captures: /ground_truth/state is world ENU, GNSS
/map is ENU, the IMU orientation is body-FLU to world-ENU. Other bags require
explicit adapters. Unregistered camera_init poses remain a separate frame.
"""
import math
from pathlib import Path
import numpy as np
from .recording import RecordingWriter
from .schema import Event

TOPICS={'/imu/data':('imu','imu'),'/msim/spoofed_gnss_odom':('gnss','gnss'),
        '/msim/normal_gnss_odom':('gnss','gnss_normal'),'/ground_truth/state':('pose','truth'),
        '/Odometry':('pose','lio'),'/loose_odom':('pose','loose'),'/velodyne_points':('cloud','lidar')}


def stamp_ns(message):
    value=message.header.stamp
    return int(value.sec)*1_000_000_000+int(value.nanosec)


def xyz(v):return [float(v.x),float(v.y),float(v.z)]
def quaternion(q):
    values=[float(q.x),float(q.y),float(q.z),float(q.w)]
    if any(not math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1.)>.02:return None
    return values


def point_fields(message):
    """Preserve all scalar numeric fields, endian, row padding and raw point time."""
    types={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
    fields=[f for f in message.fields if f.count==1 and f.datatype in types]
    if not {'x','y','z'}.issubset(f.name for f in fields):raise ValueError('PointCloud2 lacks scalar XYZ')
    order='>' if message.is_bigendian else '<'
    dtype=np.dtype(dict(names=[f.name for f in fields],formats=[order+types[f.datatype] for f in fields],
                       offsets=[f.offset for f in fields],itemsize=message.point_step))
    required=(message.height-1)*message.row_step+message.width*message.point_step if message.height else 0
    if len(message.data)<required:raise ValueError('Truncated PointCloud2')
    values=np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                      strides=(message.row_step,message.point_step))
    return {f.name:values[f.name].reshape(-1).copy() for f in fields}


def import_bag(source,destination,duration_s=None,include_clouds=False,antenna_offset=(.05,0.,.04)):
    try:from rosbags.highlevel import AnyReader
    except ImportError as error:raise ValueError('Install optional bag support: python -m pip install ".[bag]"') from error
    if duration_s is not None and (not math.isfinite(duration_s) or duration_s<=0):raise ValueError('Duration must be positive')
    if len(antenna_offset)!=3 or any(not math.isfinite(v) for v in antenna_offset):raise ValueError('Invalid antenna offset')
    path=Path(source);path=path/'sensors.bag' if path.is_dir() else path
    metadata=dict(source='msim_ros1_bag',source_file=str(path.resolve()),antenna_offset_body_m=list(antenna_offset),
        imu_orientation_reference='body_flu_to_enu',measurement_timestamps_preserved=True,
        availability_time='original bag record time; not header stamp',point_times='original scalar fields; no synthetic offsets',
        poses='precomputed bag output; FAST-LIO is not executed',clouds_included=include_clouds)
    with AnyReader([path]) as reader,RecordingWriter(destination,metadata) as writer:
        connections=[c for c in reader.connections if c.topic in TOPICS and (include_clouds or TOPICS[c.topic][0]!='cloud')]
        if not connections:raise ValueError('No supported msim topics')
        beginning=None
        for connection,arrival,raw in reader.messages(connections=connections):
            if beginning is None:beginning=arrival
            if duration_s is not None and arrival-beginning>duration_s*1e9:break
            message=reader.deserialize(raw,connection.msgtype);stamp=stamp_ns(message)
            kind,stream=TOPICS[connection.topic]
            if kind=='imu':
                data=dict(acceleration_mps2=xyz(message.linear_acceleration),angular_velocity_radps=xyz(message.angular_velocity),
                    orientation_xyzw=quaternion(message.orientation) if message.orientation_covariance[0]!=-1 else None)
                frame='body_flu'
            elif kind=='cloud':
                frame=message.header.frame_id or 'lidar'
                writer.cloud(stamp,int(arrival),stream,frame,point_fields(message));continue
            else:
                pose=message.pose.pose
                data=dict(position_m=xyz(pose.position),orientation_xyzw=quaternion(pose.orientation),
                          point='antenna' if kind=='gnss' else 'body',source_frame=message.header.frame_id)
                # Only msim's known world/map ENU streams are normalized. Do not
                # silently declare LIO's camera_init orientation to be ENU.
                frame='enu' if stream in ('truth','gnss','gnss_normal') else message.header.frame_id or 'unknown'
            writer.write(Event(stamp,int(arrival),kind,stream,frame,data))
    return writer.metadata|dict(counts=dict(writer.counts))
