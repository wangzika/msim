#!/usr/bin/env python3
"""Export a 3D/plan view of one live Gazebo + FAST_GLIO capture."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag


def position(message):
    p = message.pose.pose.position
    return p.x, p.y, p.z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True, type=Path)
    args = ap.parse_args()
    bag_path = args.run / 'sensors.bag'
    topics = {'/ground_truth/state': [], '/Odometry': [], '/loose_odom': []}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(topics=list(topics)):
            topics[topic].append((message.header.stamp.to_sec(), position(message)))
        gps_path = []
        for _, message, _ in bag.read_messages(topics=['/rtk_path']):
            gps_path = [(pose.header.stamp.to_sec(),
                         (pose.pose.position.x, pose.pose.position.y,
                          pose.pose.position.z)) for pose in message.poses]
    if not all(topics.values()) or not gps_path:
        ap.error('live FAST_GLIO trajectories are incomplete')
    anchor = topics['/ground_truth/state'][0][1]
    truth = [(stamp, tuple(p[i] - anchor[i] for i in range(3)))
             for stamp, p in topics['/ground_truth/state']]
    attack_at = None
    attack_log = args.run / 'injected-gps.jsonl'
    if attack_log.is_file():
        for line in attack_log.open():
            row = json.loads(line)
            if row.get('attack_active'):
                attack_at = row['sim_time_ns'] / 1e9
                break

    plt.rcParams.update({'font.size': 11, 'axes.labelcolor': '#dce8f4',
                         'text.color': '#dce8f4', 'xtick.color': '#aabdd0',
                         'ytick.color': '#aabdd0', 'grid.color': '#587084'})
    fig = plt.figure(figsize=(14, 7), facecolor='#101f2d')
    ax3d = fig.add_subplot(121, projection='3d', facecolor='#101f2d')
    ax2d = fig.add_subplot(122, facecolor='#101f2d')
    tracks = [('Gazebo truth', truth, '#e1e8ef', 2.0),
              ('LiDAR + IMU', topics['/Odometry'], '#47d6f3', 1.7),
              ('FAST_GLIO loose', topics['/loose_odom'], '#ff9e4a', 2.3),
              ('Injected GPS', gps_path, '#a8e368', 1.7)]
    for label, track, color, width in tracks:
        xyz = [sample[1] for sample in track]
        x, y, z = zip(*xyz)
        ax3d.plot(x, y, z, color=color, linewidth=width, label=label)
        ax2d.plot(x, y, color=color, linewidth=width, label=label)
        ax2d.scatter([x[-1]], [y[-1]], s=24, color=color)
    if attack_at is not None:
        point = min(truth, key=lambda row: abs(row[0] - attack_at))[1]
        ax2d.scatter([point[0]], [point[1]], s=90, color='#f35f73', marker='x',
                     linewidths=2.5, label='Spoof begins')
    ax3d.set(xlabel='East (m)', ylabel='North (m)', zlabel='Up (m)')
    ax2d.set(xlabel='East (m)', ylabel='North (m)')
    ax2d.axis('equal')
    ax2d.grid(alpha=0.32)
    ax3d.grid(alpha=0.25)
    ax3d.view_init(elev=28, azim=-65)
    ax3d.set_title('3D trajectory', pad=14)
    ax2d.set_title('Horizontal trajectory', pad=14)
    ax2d.legend(loc='best', facecolor='#182d40', edgecolor='#40576b')
    fig.suptitle('Gazebo + FAST_GLIO live loose fusion', fontsize=16)
    fig.text(0.5, 0.025,
             'Truth translated to its initial position; no rotation or scale fit.',
             ha='center', color='#aabdd0', fontsize=9)
    fig.tight_layout(rect=[0, 0.04, 1, 0.96])
    output = args.run / 'fastglio-live-3d.png'
    fig.savefig(output, dpi=160, facecolor=fig.get_facecolor())
    print(output)


if __name__ == '__main__':
    main()
