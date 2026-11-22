#!/usr/bin/env python3
"""Export ENU trajectories for pure LIO and GNSS loose coupling."""
import argparse
import csv
import importlib.util
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np

spec = importlib.util.spec_from_file_location(
    'comparison', Path(__file__).with_name('compare-fastglio-spoof-runs.py'))
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--clean', type=Path, required=True)
    parser.add_argument('--spoofed', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    runs = [comparison.read_run(args.clean), comparison.read_run(args.spoofed)]
    start = max(rows[0, 0] for run in runs for rows in run['errors'].values())
    end = min(rows[-1, 0] for run in runs for rows in run['errors'].values())
    attack = runs[1]['attack']
    if attack is None or end <= start:
        raise ValueError('missing attack or common time interval')
    topics = ('truth', '/Odometry', '/loose_odom')
    tracks = [{topic: run['aligned'][topic][
        (run['aligned'][topic][:, 0] >= start) &
        (run['aligned'][topic][:, 0] <= end)] for topic in topics} for run in runs]
    xyz = np.vstack([rows[:, 1:] for group in tracks for rows in group.values()])
    low, high = xyz.min(0), xyz.max(0)
    margin = np.maximum((high-low)*.06, .5)
    low, high = low-margin, high+margin
    plt.rcParams.update({'font.family': 'Noto Sans CJK SC', 'font.size': 11})
    args.output.mkdir(parents=True, exist_ok=True)

    def decorate(ax):
        ax.set(xlabel='东 E / m', ylabel='北 N / m', zlabel='天 U / m',
               xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]))
        ax.set_box_aspect(high-low)
        ax.view_init(elev=24, azim=-57)
        ax.tick_params(labelsize=9)
        ax.xaxis.set_major_locator(MaxNLocator(nbins=4))

    fig = plt.figure(figsize=(15, 8.5))
    labels = ('无欺骗飞行', '有 GPS 欺骗飞行')
    styles = {'truth': ('Gazebo 真值', '#343a40', '--'),
              '/Odometry': ('纯 LiDAR/IMU', '#168aad', '-'),
              '/loose_odom': ('FASTGLIO 松耦合', '#e67700', '-')}
    for index, group in enumerate(tracks):
        ax = fig.add_subplot(1, 2, index+1, projection='3d')
        for topic, (label, color, style) in styles.items():
            rows = group[topic]
            ax.plot(*rows[:, 1:].T, color=color, linestyle=style, linewidth=2, label=label)
            ax.scatter(*rows[0, 1:], color=color, marker='o', s=18, depthshade=False)
            ax.scatter(*rows[-1, 1:], color=color, marker='^', s=40, depthshade=False)
            if index == 1:
                point = rows[np.argmin(abs(rows[:, 0]-attack)), 1:]
                ax.scatter(*point, color=color, marker='*', s=95, depthshade=False)
        decorate(ax)
        ax.set_title(labels[index], fontsize=16, pad=18)
        ax.legend(loc='upper left', fontsize=10)
    fig.suptitle('三维轨迹对比：纯 LiDAR/IMU、FASTGLIO 松耦合与真值', fontsize=19, y=.98)
    fig.text(.5, .04, '圆点：起点；三角：终点；星形：GPS 欺骗开始。两图坐标范围一致，长度单位均为米。',
             ha='center', fontsize=11)
    fig.text(.5, .012, '两次独立飞行，各轨迹仅作首点平移对齐；纯 LiDAR/IMU 不接收 GPS。',
             ha='center', fontsize=10, color='#555555')
    fig.subplots_adjust(left=.01, right=.97, top=.86, bottom=.10, wspace=.05)
    fig.savefig(args.output / 'lio-loose-3d-comparison.png', dpi=200)
    fig.savefig(args.output / 'lio-loose-3d-comparison.pdf')
    plt.close(fig)

    fig = plt.figure(figsize=(11, 10))
    ax = fig.add_subplot(projection='3d')
    for index, (group, color) in enumerate(zip(tracks, ('#168aad', '#d6336c'))):
        for topic, style, method in (('truth', ':', '真值'),
                                     ('/Odometry', '--', '纯 LiDAR/IMU'),
                                     ('/loose_odom', '-', '松耦合')):
            rows = group[topic]
            ax.plot(*rows[:, 1:].T, color=color, linestyle=style, linewidth=1.8,
                    label=f'{labels[index]} · {method}')
    decorate(ax)
    ax.legend(loc='upper left', fontsize=10)
    ax.set_title('两次飞行的三维轨迹叠加对比', fontsize=18, pad=25)
    fig.text(.5, .025, '虚线：纯 LiDAR/IMU；实线：松耦合；点线：对应飞行真值。', ha='center', fontsize=11)
    fig.subplots_adjust(top=.88, bottom=.07, left=0, right=.95)
    fig.savefig(args.output / 'lio-loose-3d-overlay.png', dpi=180)
    fig.savefig(args.output / 'lio-loose-3d-overlay.pdf')
    plt.close(fig)

    with (args.output / 'lio-loose-3d-trajectories.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(('run', 'series', 'relative_time_s', 'east_m', 'north_m', 'up_m'))
        for label, group in zip(('clean', 'spoofed'), tracks):
            for topic, rows in group.items():
                for row in rows:
                    writer.writerow((label, topic, *row))
    print(args.output / 'lio-loose-3d-comparison.png')


if __name__ == '__main__':
    main()
