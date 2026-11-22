#!/usr/bin/env python3
"""Plot live FAST_GLIO east/north/up displacement errors against Gazebo truth."""
import argparse
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag


TOPICS = {
    '/Odometry': ('纯 LiDAR/IMU', '#43d5f4'),
    '/loose_odom': ('FAST_GLIO 松组合', '#ff9d4c'),
    '/msim/spoofed_gnss_odom': ('注入 GPS', '#a6df62'),
}
ANTENNA_BODY = (0.05, 0.0, 0.04)


def stamp(message):
    return message.header.stamp.to_sec()


def position(message):
    p = message.pose.pose.position
    return p.x, p.y, p.z


def antenna_position(message):
    """Gazebo body truth transformed to the GNSS antenna for GPS comparison."""
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    x, y, z = ANTENNA_BODY
    tx, ty, tz = 2 * (q.y*z - q.z*y), 2 * (q.z*x - q.x*z), 2 * (q.x*y - q.y*x)
    return (p.x + x + q.w*tx + q.y*tz - q.z*ty,
            p.y + y + q.w*ty + q.z*tx - q.x*tz,
            p.z + z + q.w*tz + q.x*ty - q.y*tx)


def nearest_truth(rows, timestamps, when, max_delta_s=0.05):
    index = bisect_left(timestamps, when)
    candidates = [rows[i] for i in (index - 1, index) if 0 <= i < len(rows)]
    if not candidates:
        return None
    nearest = min(candidates, key=lambda item: abs(item[0] - when))
    return nearest if abs(nearest[0] - when) <= max_delta_s else None


def errors_for(track, truth, truth_timestamps, gps=False):
    rows = []
    anchor = None
    for when, estimate in track:
        match = nearest_truth(truth, truth_timestamps, when)
        if match is None:
            continue
        reference = antenna_position(match[1]) if gps else position(match[1])
        difference = tuple(estimate[i] - reference[i] for i in range(3))
        if anchor is None:
            anchor = difference
        rows.append((when, *[difference[i] - anchor[i] for i in range(3)]))
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', required=True, type=Path)
    args = parser.parse_args()
    run = args.run
    truth = []
    tracks = {topic: [] for topic in TOPICS}
    with rosbag.Bag(str(run / 'sensors.bag')) as bag:
        for topic, message, _ in bag.read_messages(
                topics=['/ground_truth/state', *TOPICS]):
            if topic == '/ground_truth/state':
                truth.append((stamp(message), message))
            else:
                tracks[topic].append((stamp(message), position(message)))
    if not truth or not all(tracks.values()):
        parser.error('truth or FAST_GLIO trajectories missing from sensors.bag')
    truth.sort(key=lambda item: item[0])
    truth_timestamps = [item[0] for item in truth]
    errors = {topic: errors_for(track, truth, truth_timestamps,
                                gps=topic == '/msim/spoofed_gnss_odom')
              for topic, track in tracks.items()}
    if any(len(rows) < 10 for rows in errors.values()):
        parser.error('too few trajectory samples matched truth within 50 ms')
    origin = truth[0][0]
    attack_at = None
    log_path = run / 'injected-gps.jsonl'
    if log_path.is_file():
        first_record = True
        for line in log_path.open():
            row = json.loads(line)
            if first_record:
                origin = row['sim_time_ns'] / 1e9
                first_record = False
            if row.get('attack_active'):
                attack_at = row['sim_time_ns'] / 1e9 - origin
                break

    plt.rcParams.update({'font.family': 'Noto Sans CJK SC', 'font.size': 11,
                         'axes.labelcolor': '#e3ebf4', 'text.color': '#e3ebf4',
                         'xtick.color': '#b9cada', 'ytick.color': '#b9cada',
                         'grid.color': '#526b7e'})
    fig, axes = plt.subplots(3, 1, figsize=(12.5, 9), sharex=True,
                             facecolor='#101f2d')
    axis_names = ('东向 E', '北向 N', '天向 U')
    for index, ax in enumerate(axes):
        ax.set_facecolor('#101f2d')
        ax.axhline(0, color='#8195a8', linewidth=1, alpha=0.8)
        if attack_at is not None:
            ax.axvspan(attack_at, truth[-1][0]-origin, color='#ef5c70', alpha=0.07)
            ax.axvline(attack_at, color='#ef5c70', linestyle='--', linewidth=1.3,
                       label='欺骗开始' if index == 0 else None)
        for topic, (label, color) in TOPICS.items():
            rows = errors[topic]
            ax.plot([row[0]-origin for row in rows],
                    [row[index+1] for row in rows],
                    color=color, linewidth=1.8, label=label if index == 0 else None)
        ax.set_ylabel(f'{axis_names[index]}误差 (m)')
        ax.grid(alpha=0.33)
        for spine in ax.spines.values():
            spine.set_color('#466073')
    axes[0].legend(loc='upper left', ncol=2, facecolor='#1a3042',
                   edgecolor='#466073')
    axes[-1].set_xlabel('相对 GPS 注入开始的仿真时间 (s)'
                        if log_path.is_file() else '相对首帧真值的仿真时间 (s)')
    fig.suptitle('Gazebo 真值参考的 ENU 时序误差', fontsize=17, y=0.985)
    fig.text(0.5, 0.018,
             '误差 = 估计位移 − 真值位移；各轨迹以首个匹配点平移对齐，未做旋转或尺度拟合。',
             ha='center', color='#b9cada', fontsize=10)
    fig.tight_layout(rect=(0, 0.035, 1, 0.965))
    output = run / 'fastglio-enu-error-timeseries.png'
    fig.savefig(output, dpi=170, facecolor=fig.get_facecolor())
    plt.close(fig)

    summary = {}
    for topic, rows in errors.items():
        name = {'/Odometry': 'lidar_imu', '/loose_odom': 'loose',
                '/msim/spoofed_gnss_odom': 'injected_gps'}[topic]
        summary[name] = dict(samples=len(rows),
                             rmse_enu_m=[math.sqrt(sum(row[i]**2 for row in rows)
                                                   / len(rows)) for i in (1, 2, 3)],
                             final_enu_error_m=list(rows[-1][1:]))
    summary['method'] = 'first matched point translation; no rotation or scale fit'
    summary['truth_match_limit_ms'] = 50
    summary['time_origin_sim_s'] = origin
    summary['attack_start_relative_s'] = attack_at
    (run / 'fastglio-enu-error-summary.json').write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
    with (run / 'fastglio-enu-error-timeseries.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(('series', 'sim_time_s', 'relative_time_s',
                         'east_error_m', 'north_error_m', 'up_error_m'))
        for topic, rows in errors.items():
            for row in rows:
                writer.writerow((topic, row[0], row[0]-origin, *row[1:]))
    print(output)


if __name__ == '__main__':
    main()
