#!/usr/bin/env python3
"""Accumulate registered LIO clouds into a bounded voxel map and save a PCD."""
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

import numpy as np


class VoxelMap:
    def __init__(self, resolution=.4, maximum=1_000_000):
        if not math.isfinite(resolution) or resolution <= 0 or maximum < 1:
            raise ValueError('Invalid map limits')
        self.resolution = resolution
        self.maximum = maximum
        self.points = {}
        self.capped = False

    def add(self, points):
        points = np.asarray(points, dtype=np.float32).reshape(-1, 4)
        points = points[np.isfinite(points).all(1)]
        if not len(points):return
        cells = np.floor(points[:, :3]/self.resolution).astype(np.int64)
        _, index = np.unique(cells, axis=0, return_index=True)
        for cell, point in zip(cells[index], points[index]):
            key = tuple(cell)
            if key not in self.points:
                if len(self.points) >= self.maximum:
                    self.capped = True
                    continue
                self.points[key] = tuple(point)

    def array(self):
        return np.asarray(list(self.points.values()), dtype='<f4').reshape(-1, 4)

    def save(self, path):
        points = self.array()
        header = (f'# .PCD v0.7 - accumulated LiDAR/IMU map in camera_init\nVERSION 0.7\n'
                  f'FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
                  f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n')
        path = Path(path); temporary = path.with_suffix('.pcd.tmp')
        with temporary.open('wb') as stream:
            stream.write(header.encode());stream.write(points.tobytes())
        temporary.replace(path)


def cloud_points(message):
    fields = {f.name:f for f in message.fields}
    keys = [k for k in ('x', 'y', 'z', 'intensity') if k in fields]
    if any(k not in keys for k in ('x','y','z')) or any(fields[k].datatype != 7 for k in keys):
        raise ValueError('Expected float32 XYZ registered cloud')
    order = '>' if message.is_bigendian else '<'
    dtype = np.dtype(dict(names=keys, formats=[order+'f4']*len(keys),
                         offsets=[fields[k].offset for k in keys], itemsize=message.point_step))
    rows = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                      strides=(message.row_step, message.point_step))
    result = np.zeros((message.height*message.width,4), dtype=np.float32)
    for i,k in enumerate(('x','y','z','intensity')):
        if k in keys:result[:,i] = rows[k].ravel()
    return result


def main():
    import rospy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
    rospy.init_node('msim_lio_map_recorder', disable_signals=True)
    run = Path(os.environ['MSIM_TELEOP_RUN'])
    grid = VoxelMap(float(os.environ.get('MSIM_MAP_VOXEL_M', '.4')))
    lock = threading.Lock(); state = dict(clouds=0, frame=None, stamp=None)
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop);signal.signal(signal.SIGTERM, stop)
    def receive(msg):
        if msg.header.frame_id != 'camera_init':
            rospy.logerr_throttle(5, 'Refusing cloud outside camera_init: %s', msg.header.frame_id)
            return
        points = cloud_points(msg)
        with lock:
            grid.add(points);state.update(clouds=state['clouds']+1,frame=msg.header.frame_id,stamp=msg.header.stamp)
    subscriber = rospy.Subscriber('/cloud_registered', PointCloud2, receive, queue_size=3,
                                  buff_size=8*1024*1024)
    publisher = rospy.Publisher('/msim/lio_map', PointCloud2, queue_size=1, latch=True)
    status = rospy.Publisher('/msim/lio_map/status', String, queue_size=1, latch=True)
    fields = []
    for i,name in enumerate(('x','y','z','intensity')):
        f=PointField();f.name=name;f.offset=i*4;f.datatype=PointField.FLOAT32;f.count=1;fields.append(f)
    last_save = time.monotonic()
    def report(final=False):
        with lock:
            row = dict(clouds=state['clouds'], points=len(grid.points), frame_id=state['frame'],
                       voxel_size_m=grid.resolution, maximum_voxels=grid.maximum, capped=grid.capped,
                       final=final, source_topic='/cloud_registered', output_topic='/msim/lio_map',
                       map_type='Voxel accumulation of registered LIO scans; no loop closure or global optimization')
        temporary=run/'mapping-report.json.tmp';temporary.write_text(json.dumps(row,indent=2)+'\n');temporary.replace(run/'mapping-report.json')
        status.publish(String(data=json.dumps(row)))
    try:
        while running and not rospy.is_shutdown():
            with lock:
                points=grid.array();stamp=state['stamp']
            if stamp is not None:
                msg=PointCloud2();msg.header.frame_id='camera_init';msg.header.stamp=stamp
                msg.height=1;msg.width=len(points);msg.fields=fields;msg.point_step=16
                msg.row_step=16*len(points);msg.is_dense=True;msg.data=points.tobytes()
                publisher.publish(msg)
            if time.monotonic()-last_save > 30:
                with lock:grid.save(run/'map.pcd')
                last_save=time.monotonic()
            report()
            time.sleep(.5)
    finally:
        subscriber.unregister()
        with lock:grid.save(run/'map.pcd')
        report(final=True)
        print(f'地图已保存：{run / "map.pcd"}',flush=True)


if __name__ == '__main__':main()
