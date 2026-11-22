#!/usr/bin/env python3
"""Add Velodyne ring/time fields to the Gazebo 16-beam snapshot cloud.

Gazebo's ray sensor supplies a simultaneous scan, so every point receives a
small positive offset (1 us) only to signal that the scan has point times.
This is a format adapter, not a physical spinning-LiDAR timing model.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2, PointField


OUTPUT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 16, 20, 24],
    'itemsize': 32,
})
OUTPUT_TOPIC = '/msim/velodyne_points_annotated'


def annotate(cloud):
    # New scanner publishes native ring/time fields. Preserve physical timing.
    if {'ring', 'time'} <= {field.name for field in cloud.fields}:
        return cloud
    source_dtype = np.dtype({
        'names': ['x', 'y', 'z', 'intensity'],
        'formats': ['<f4'] * 4,
        'offsets': [next(field.offset for field in cloud.fields if field.name == name)
                    for name in ('x', 'y', 'z', 'intensity')],
        'itemsize': cloud.point_step,
    })
    source = np.frombuffer(cloud.data, dtype=source_dtype,
                           count=cloud.width * cloud.height)
    valid = np.isfinite(source['x']) & np.isfinite(source['y']) & \
        np.isfinite(source['z']) & np.isfinite(source['intensity'])
    source = source[valid]
    output = np.zeros(len(source), dtype=OUTPUT_DTYPE)
    for name in ('x', 'y', 'z', 'intensity'):
        output[name] = source[name]
    horizontal = np.hypot(source['x'], source['y'])
    elevation_deg = np.degrees(np.arctan2(source['z'], horizontal))
    output['ring'] = np.clip(np.rint((elevation_deg + 15.0) / 2.0),
                             0, 15).astype(np.uint16)
    output['time'] = 1e-6
    result = PointCloud2()
    result.header = cloud.header
    result.height = 1
    result.width = len(output)
    result.fields = [PointField(name, OUTPUT_DTYPE.fields[name][1],
                                PointField.UINT16 if name == 'ring' else PointField.FLOAT32,
                                1) for name in OUTPUT_DTYPE.names]
    result.is_bigendian = False
    result.point_step = OUTPUT_DTYPE.itemsize
    result.row_step = result.point_step * result.width
    result.data = output.tobytes()
    result.is_dense = True
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', required=True, type=Path)
    args = parser.parse_args()
    output_dir = args.run / 'comparisons'
    output_dir.mkdir(exist_ok=True)
    output_path = output_dir / 'annotated-sensors.bag'
    counts = {'/imu/data': 0, '/ground_truth/state': 0, OUTPUT_TOPIC: 0}
    with rosbag.Bag(str(args.run / 'sensors.bag')) as source, \
            rosbag.Bag(str(output_path), 'w', compression='lz4') as target:
        for topic, message, stamp in source.read_messages(
                topics=['/imu/data', '/ground_truth/state', '/velodyne_points']):
            if topic == '/velodyne_points':
                message = annotate(message)
                topic = OUTPUT_TOPIC
            target.write(topic, message, stamp)
            counts[topic] += 1
    manifest = {
        'source': str(args.run / 'sensors.bag'),
        'output': str(output_path),
        'method': 'ring from Gazebo vertical angle -15..15 deg across 16 beams; '
                  'uniform 1 us point offset for instantaneous ray snapshot',
        'counts': counts,
    }
    (output_dir / 'annotated-sensors.json').write_text(
        json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
