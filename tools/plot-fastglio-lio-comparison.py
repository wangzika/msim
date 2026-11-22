#!/usr/bin/env python3
"""Plot translation-only ENU errors for the same Gazebo LiDAR/IMU recording."""
import argparse
from bisect import bisect_left
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag


def samples(path, topic):
    tracks = {topic: [], '/ground_truth/state': []}
    with rosbag.Bag(str(path)) as bag:
        for name, msg, _ in bag.read_messages(topics=list(tracks)):
            p = msg.pose.pose.position
            tracks[name].append((msg.header.stamp.to_sec(), np.array([p.x, p.y, p.z])))
    return tracks[topic], tracks['/ground_truth/state']


def errors(path, topic):
    odom, truth = samples(path, topic)
    truth_times = [item[0] for item in truth]
    matched = []
    for t, pos in odom:
        i = bisect_left(truth_times, t)
        closest = min((truth[j] for j in (i - 1, i) if 0 <= j < len(truth)),
                      key=lambda item: abs(item[0] - t), default=None)
        if closest is not None and abs(closest[0] - t) <= 0.05:
            matched.append((t, pos, closest[1]))
    if len(matched) < 10:
        return None
    origin_odom, origin_truth = matched[0][1:]
    return (np.array([m[0] for m in matched]),
            np.array([m[1] - origin_odom - (m[2] - origin_truth) for m in matched]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True, type=Path)
    args = ap.parse_args()
    run = args.run
    paths = [
        ('FAST-LIO', run / 'combined-signalsim.bag', '/Odometry', '#2460ab'),
        ('FAST-LIO Velodyne input', run / 'comparisons/fast-lio-velodyne/algorithm.bag',
         '/Odometry', '#6a63b8'),
        ('GLIO front-end', run / 'comparisons/glio-front-end/algorithm.bag', '/odom', '#dd8c19'),
        ('LIGO LIO', run / 'comparisons/ligo/algorithm.bag', '/Odometry', '#14945d'),
        ('LIO-SAM (patched)', run / 'comparisons/lio-sam-patched/algorithm.bag',
         '/lio_sam/mapping/odometry', '#bb3f7b'),
    ]
    start = 4.866
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for label, path, topic, color in paths:
        if not path.exists():
            continue
        result = errors(path, topic)
        if result is None:
            continue
        times, delta = result
        for axis, column in zip(axes, range(3)):
            axis.plot(times - start, delta[:, column], label=label, color=color,
                      linewidth=1.7)
    for axis, name in zip(axes, ('East', 'North', 'Up')):
        axis.axhline(0, color='#777777', linestyle=':', linewidth=0.9)
        axis.set_ylabel(f'{name} error (m)')
        axis.grid(alpha=0.25)
        axis.legend(loc='best', fontsize=8)
    axes[-1].set_xlabel('Time since recording start (s)')
    fig.suptitle('Pure LiDAR/IMU odometry vs Gazebo truth | first-pose translation alignment')
    fig.tight_layout()
    target = run / 'comparisons/lio-enu-errors.png'
    fig.savefig(target, dpi=180)
    print(target)

    ablation = [
        ('Current MARSIM, all points', run / 'combined-signalsim.bag', '#2460ab'),
        ('Velodyne, all points',
         run / 'comparisons/fast-lio-velodyne-full/algorithm.bag', '#6a63b8'),
        ('Velodyne, every 2nd point',
         run / 'comparisons/fast-lio-velodyne/algorithm.bag', '#14945d'),
    ]
    fig2, axes2 = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    for label, path, color in ablation:
        if not path.exists():
            continue
        result = errors(path, '/Odometry')
        if result is None:
            continue
        times, delta = result
        for axis, column in zip(axes2, (1, 2)):
            axis.plot(times - start, delta[:, column], label=label,
                      color=color, linewidth=1.8)
    for axis, name in zip(axes2, ('North', 'Up')):
        axis.axhline(0, color='#777777', linestyle=':', linewidth=0.9)
        axis.set_ylabel(f'{name} error (m)')
        axis.grid(alpha=0.25)
        axis.legend(loc='best', fontsize=8)
    axes2[-1].set_xlabel('Time since recording start (s)')
    fig2.suptitle('FAST-LIO input and point-count ablation | same Gazebo recording')
    fig2.tight_layout()
    ablation_target = run / 'comparisons/lio-filter-ablation.png'
    fig2.savefig(ablation_target, dpi=180)
    print(ablation_target)

    orientation_runs = [
        ('Gazebo IMU quaternion',
         run / 'comparisons/lio-sam-patched/algorithm.bag', '#bb3f7b'),
        ('Same gyro integrated from initial attitude',
         run / 'comparisons/lio-sam-gyro-orientation/algorithm.bag', '#5942a7'),
    ]
    fig3, axes3 = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    for label, path, color in orientation_runs:
        if not path.exists():
            continue
        result = errors(path, '/lio_sam/mapping/odometry')
        if result is None:
            continue
        times, delta = result
        for axis, column in zip(axes3, (1, 2)):
            axis.plot(times - start, delta[:, column], label=label,
                      color=color, linewidth=1.8)
    for axis, name in zip(axes3, ('North', 'Up')):
        axis.axhline(0, color='#777777', linestyle=':', linewidth=0.9)
        axis.set_ylabel(f'{name} error (m)')
        axis.grid(alpha=0.25)
        axis.legend(loc='best', fontsize=8)
    axes3[-1].set_xlabel('Time since recording start (s)')
    fig3.suptitle('LIO-SAM IMU attitude ablation | same gyro, accel, and LiDAR')
    fig3.tight_layout()
    orientation_target = run / 'comparisons/lio-orientation-ablation.png'
    fig3.savefig(orientation_target, dpi=180)
    print(orientation_target)


if __name__ == '__main__':
    main()
