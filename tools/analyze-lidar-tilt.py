#!/usr/bin/env python3
"""Compare ground coverage and ENU errors across the controlled tilt flights."""
import argparse
from bisect import bisect_left
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation


def stamped_pose(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (msg.header.stamp.to_sec(),
            np.array([p.x, p.y, p.z]),
            Rotation.from_quat([q.x, q.y, q.z, q.w]))


def closest(truth, times, stamp):
    i = bisect_left(times, stamp)
    return min((truth[j] for j in (i - 1, i) if 0 <= j < len(truth)),
               key=lambda item: abs(item[0] - stamp), default=None)


def cloud_xyz(cloud):
    offsets = {field.name: field.offset for field in cloud.fields}
    dtype = np.dtype({'names': ('x', 'y', 'z'),
                      'formats': ('<f4', '<f4', '<f4'),
                      'offsets': tuple(offsets[name] for name in ('x', 'y', 'z')),
                      'itemsize': cloud.point_step})
    rows = np.frombuffer(cloud.data, dtype=dtype,
                         count=cloud.width * cloud.height)
    return np.column_stack([rows[name] for name in ('x', 'y', 'z')])


def analyze_case(group, angle):
    label = f'forward-down-{angle}'
    run = group / label
    with rosbag.Bag(str(run / 'sensors.bag')) as bag:
        truth = [stamped_pose(msg) for _, msg, _ in
                 bag.read_messages(topics=['/ground_truth/state'])]
        clouds = [msg for _, msg, _ in
                  bag.read_messages(topics=['/velodyne_points'])]
    truth_times = [row[0] for row in truth]
    # This is the sensor-to-body rotation used in the matching FAST-LIO config.
    lidar_to_body = Rotation.from_euler('x', -angle, degrees=True)
    low_counts, far_north_counts, total_counts = [], [], []
    for cloud in clouds:
        pose = closest(truth, truth_times, cloud.header.stamp.to_sec())
        if pose is None or abs(pose[0] - cloud.header.stamp.to_sec()) > 0.05:
            continue
        if not 7.5 <= pose[1][2] <= 8.5:
            continue
        points = cloud_xyz(cloud)
        world = pose[2].apply(lidar_to_body.apply(points) + [0, 0, 0.15]) + pose[1]
        low_counts.append(int(np.count_nonzero(
            (-0.2 < world[:, 2]) & (world[:, 2] < 0.6))))
        far_north_counts.append(int(np.count_nonzero(
            (world[:, 1] > 30) & (np.abs(world[:, 0]) < 12))))
        total_counts.append(len(points))
    odom_topic = '/Odometry'
    with rosbag.Bag(str(run / 'comparisons/fast-lio-velodyne/algorithm.bag')) as bag:
        odom = [stamped_pose(msg) for _, msg, _ in bag.read_messages(topics=[odom_topic])]
    matched = []
    for stamp, pos, _ in odom:
        pose = closest(truth, truth_times, stamp)
        if pose is not None and abs(pose[0] - stamp) <= 0.05:
            matched.append((stamp, pos, pose[1]))
    if len(matched) < 10:
        raise RuntimeError(f'{label}: insufficient matched odometry')
    origin_est, origin_truth = matched[0][1:]
    times = np.array([row[0] - matched[0][0] for row in matched])
    error = np.array([row[1] - origin_est - (row[2] - origin_truth)
                      for row in matched])
    rmse = np.sqrt(np.mean(error * error, axis=0))
    path = np.array([row[1] for row in truth])
    result = dict(angle_deg=angle, scan_count=len(clouds),
                  cruise_scan_count=len(low_counts),
                  median_low_points_per_scan=float(np.median(low_counts)),
                  median_far_north_points_per_scan=float(np.median(far_north_counts)),
                  median_total_points_per_scan=float(np.median(total_counts)),
                  low_point_fraction=float(np.sum(low_counts)/np.sum(total_counts)),
                  truth_path_length_m=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                  truth_final_north_displacement_m=float(path[-1, 1] - path[0, 1]),
                  matched_odom=len(matched),
                  enu_rmse_m=rmse.tolist(),
                  three_d_rmse_m=float(np.sqrt(np.mean(np.sum(error*error, axis=1)))),
                  final_enu_error_m=error[-1].tolist())
    return result, times, error


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--group', required=True, type=Path)
    args = ap.parse_args()
    group = args.group.resolve()
    results = []
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    colors = ('#2460ab', '#7240a1', '#14945d', '#c47122')
    for angle, color in zip((0, 5, 10, 15), colors):
        result, times, error = analyze_case(group, angle)
        results.append(result)
        for axis, column in zip(axes, (1, 2)):
            axis.plot(times, error[:, column], label=f'{angle}° forward-down',
                      color=color, linewidth=1.7)
    for axis, name in zip(axes, ('North', 'Up')):
        axis.axhline(0, color='#777', linestyle=':', linewidth=0.9)
        axis.set_ylabel(f'{name} error (m)')
        axis.legend(loc='best')
        axis.grid(alpha=0.25)
    axes[-1].set_xlabel('Time since first matched scan (s)')
    fig.suptitle('FAST-LIO pure LiDAR/IMU | LiDAR mounting tilt experiment')
    fig.tight_layout()
    target = group / 'tilt-enu-errors.png'
    fig.savefig(target, dpi=180)
    (group / 'tilt-comparison.json').write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))
    print(target)


if __name__ == '__main__':
    main()
