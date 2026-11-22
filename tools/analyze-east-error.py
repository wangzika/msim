#!/usr/bin/env python3
"""Evaluate paired local-map experiments without fitting rotation or scale."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'runs/east-error-diagnosis-20261122'
SOURCE = ROOT / 'runs/lidar-comparison-20261122-01'


def poses(bag):
    rows = {'/Odometry': [], '/ground_truth/state': []}
    with rosbag.Bag(str(bag)) as stream:
        for topic, msg, _ in stream.read_messages(topics=list(rows)):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            rows[topic].append([msg.header.stamp.to_sec(), p.x, p.y, p.z,
                                q.x, q.y, q.z, q.w])
    odom, truth = [np.array(sorted(rows[k])) for k in rows]
    odom = odom[(odom[:, 0] >= truth[0, 0]) & (odom[:, 0] <= truth[-1, 0])]
    i = np.clip(np.searchsorted(truth[:, 0], odom[:, 0]), 1, len(truth)-1)
    odom = odom[(truth[i, 0]-truth[i-1, 0]) <= .05]
    ref = np.column_stack([np.interp(odom[:, 0], truth[:, 0], truth[:, k])
                           for k in (1, 2, 3)])
    error = odom[:, 1:4]-ref
    error -= error[0]
    rt = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:]))(odom[:, 0])
    # Express the relative rotation vector in world axes (E, N, U).
    attitude = (Rotation.from_quat(odom[:, 4:])*rt.inv()).as_rotvec()*180/np.pi
    return np.column_stack([odom[:, 0], ref, error, attitude])


def stats(data, lo, hi):
    d = data[(data[:, 0] >= lo) & (data[:, 0] <= hi)]
    err = d[:, 4:7]
    return dict(samples=len(d), interval_sim_s=[float(d[0, 0]), float(d[-1, 0])],
                rmse_enu_m=np.sqrt(np.mean(err**2, axis=0)).tolist(),
                rmse_3d_m=float(np.sqrt(np.mean(np.sum(err**2, axis=1)))),
                mean_enu_m=err.mean(axis=0).tolist(),
                mean_attitude_error_deg=d[:, 7:10].mean(axis=0).tolist())


def main():
    plt.rcParams.update({'font.family': 'Noto Sans CJK SC', 'axes.unicode_minus': False})
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True)
    summary = dict(method='truth interpolated at header stamp; first matched translation only; no rotation/scale fitting',
                   caveat='paired replays start after bag recording; initial pre-recording IMU is absent, so baseline is not numerically identical to original live run',
                   runs={})
    for col, (radar, prefix, title) in enumerate([('vlp16', 'vlp', 'VLP-16'),
                                                ('os1-64', 'os1', 'OS1-64')]):
        data = {mode: poses(OUT / f'{prefix}-{mode}' / 'algorithm.bag')
                for mode in ('baseline', 'cube1000')}
        lo = max(d[0, 0] for d in data.values())
        hi = min(d[-1, 0] for d in data.values())
        entry = dict(common_sim_interval_s=[lo, hi], cases={})
        for mode, label, color in [('baseline', '原参数（实际 200 m）', '#D64B5D'),
                                    ('cube1000', '修正参数（1000 m）', '#227DA8')]:
            d = data[mode]
            m = stats(d, lo, hi)
            m['ground_4_to_8s'] = stats(d, 4, 8)
            m['flight_14_to_60s'] = stats(d, 14, 60)
            path = OUT / f'{prefix}-{mode}' / 'matching.csv'
            matching = list(csv.DictReader(path.open()))
            m['geometry_4_to_8s'] = {k: float(np.mean([float(x[k]) for x in matching
                                                       if 4 <= float(x['stamp_s']) <= 8]))
                                      for k in ('normal_E', 'normal_N', 'info_E', 'info_N',
                                                'map_points', 'line_neighborhood_fraction')}
            entry['cases'][mode] = m
            np.savetxt(OUT / f'{prefix}-{mode}-evaluated.csv', d, delimiter=',', comments='',
                       header='sim_time,true_E,true_N,true_U,error_E,error_N,error_U,attitude_x_deg,attitude_y_deg,attitude_z_deg')
            for row, (idx, ylabel) in enumerate([(4, '东向误差 / m'),
                                                 (5, '北向误差 / m'),
                                                 (8, '绕北轴姿态误差 / °')]):
                axes[row, col].plot(d[:, 0], d[:, idx], label=label, color=color, lw=2)
                axes[row, col].set_ylabel(ylabel, fontsize=19)
        entry['east_rmse_reduction_fraction'] = 1-entry['cases']['cube1000']['rmse_enu_m'][0]/entry['cases']['baseline']['rmse_enu_m'][0]
        summary['runs'][radar] = entry
        axes[0, col].set_title(title, fontsize=23, pad=12)
        axes[-1, col].set_xlabel('仿真时间 / s', fontsize=20)
    for ax in axes.flat:
        ax.axhline(0, color='#8A9299', lw=.7)
        ax.grid(color='#E4E7EA');ax.tick_params(labelsize=16)
        ax.spines[['top', 'right']].set_visible(False)
        ax.set_xlim(2, 63)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', ncol=2, fontsize=18, frameon=False)
    fig.subplots_adjust(left=.11, right=.98, top=.9, bottom=.09, wspace=.31, hspace=.24)
    fig.savefig(OUT/'map-size-control.png', dpi=220, facecolor='white');plt.close(fig)
    (OUT/'paired-results.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    for radar, r in summary['runs'].items():
        print(radar, r['common_sim_interval_s'])
        for mode, m in r['cases'].items():
            print(mode, 'RMSE ENU', m['rmse_enu_m'], '3D', m['rmse_3d_m'],
                  'ground attitude', m['ground_4_to_8s']['mean_attitude_error_deg'])


if __name__ == '__main__':
    main()
