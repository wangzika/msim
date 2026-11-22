#!/usr/bin/env python3
"""Compare two closed-loop captures using each flight's own Gazebo truth."""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag

spec = importlib.util.spec_from_file_location(
    'enu', Path(__file__).with_name('plot-fastglio-enu-error.py'))
enu = importlib.util.module_from_spec(spec)
spec.loader.exec_module(enu)

SERIES = {
    '/Odometry': ('LiDAR + IMU', '#168aad'),
    '/loose_odom': ('FASTGLIO 松耦合', '#e67700'),
    '/msim/spoofed_gnss_odom': ('输入 GPS', '#5c940d'),
}


def read_run(path):
    truth, tracks = [], {topic: [] for topic in SERIES}
    with rosbag.Bag(str(path / 'sensors.bag')) as bag:
        for topic, msg, _ in bag.read_messages(topics=['/ground_truth/state', *SERIES]):
            if topic == '/ground_truth/state':
                truth.append((enu.stamp(msg), msg))
            else:
                tracks[topic].append((enu.stamp(msg), enu.position(msg)))
    truth.sort(key=lambda row: row[0])
    times = [row[0] for row in truth]
    if not truth or any(not values for values in tracks.values()):
        raise ValueError(f'missing trajectory in {path}')
    logs = [json.loads(line) for line in (path / 'injected-gps.jsonl').read_text().splitlines()]
    origin = logs[0]['sim_time_ns'] / 1e9
    attack = next((row['sim_time_ns'] / 1e9 - origin
                   for row in logs if row.get('attack_active')), None)
    anchor = np.array(enu.position(truth[0][1]))
    aligned = {'truth': np.array([(t-origin, *(np.array(enu.position(m))-anchor))
                                  for t, m in truth])}
    errors = {}
    for topic, track in tracks.items():
        gps = topic == '/msim/spoofed_gnss_odom'
        errors[topic] = np.array(enu.errors_for(track, truth, times, gps=gps))
        errors[topic][:, 0] -= origin
        first = next((item for item in track if enu.nearest_truth(truth, times, item[0])), None)
        ref = enu.nearest_truth(truth, times, first[0])[1]
        reference = enu.antenna_position(ref) if gps else enu.position(ref)
        offset = np.array(first[1])-np.array(reference)
        aligned[topic] = np.array([(t-origin, *(np.array(p)-offset-anchor)) for t, p in track])
    return dict(path=path, logs=logs, attack=attack, aligned=aligned, errors=errors,
                replay_manifest=(json.loads((path / 'replay-manifest.json').read_text())
                                 if (path / 'replay-manifest.json').exists() else None),
                flight_report=json.loads((path / 'px4-flight-report.json').read_text()),
                loose_report=json.loads((path / 'fastglio-live-report.json').read_text()))


def metrics(rows):
    if not len(rows):
        return None
    err = rows[:, 1:]
    return dict(samples=len(rows), rmse_enu_m=np.sqrt(np.mean(err**2, axis=0)).tolist(),
                rmse_3d_m=float(np.sqrt(np.mean(np.sum(err**2, axis=1)))),
                final_enu_error_m=err[-1].tolist(),
                max_3d_error_m=float(np.max(np.linalg.norm(err, axis=1))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--clean', required=True, type=Path)
    parser.add_argument('--spoofed', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    runs = [read_run(args.clean), read_run(args.spoofed)]
    for filename in ('sensor_profile.json', 'route_plan.json'):
        if (args.clean / filename).read_bytes() != (args.spoofed / filename).read_bytes():
            raise ValueError(f'comparison inputs differ: {filename}')
    args.output.mkdir(parents=True, exist_ok=True)
    labels = ['无欺骗', '有 GPS 欺骗']
    plt.rcParams.update({'font.family': 'Noto Sans CJK SC', 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False})
    start = max(rows[0, 0] for run in runs for rows in run['errors'].values())
    end = min(rows[-1, 0] for run in runs for rows in run['errors'].values())
    if end <= start:
        raise ValueError('runs have no common relative-time interval')
    attack = runs[1]['attack']
    if attack is None:
        raise ValueError('spoofed run contains no active attack')

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True, sharey='row')
    for col, run in enumerate(runs):
        for axis, name in enumerate(('东向 E', '北向 N', '天向 U')):
            ax = axes[axis, col]
            ax.axhline(0, color='#adb5bd', linewidth=0.8)
            for topic, (label, color) in SERIES.items():
                rows = run['errors'][topic]
                ax.plot(rows[:, 0], rows[:, axis+1], color=color, label=label, linewidth=1.4,
                        linestyle='--' if topic.endswith('gnss_odom') else '-')
            if col == 1:
                ax.axvline(attack, color='#d6336c', linestyle='--', linewidth=1.2)
                ax.axvspan(attack, end, color='#d6336c', alpha=0.06)
            ax.set_ylabel(f'{name}误差 / m')
            ax.set_xlim(start, end)
            ax.grid(alpha=0.25)
        axes[0, col].set_title(labels[col], fontsize=15)
        axes[0, col].legend(fontsize=10)
        axes[-1, col].set_xlabel('相对 GPS 源启动的仿真时间 / s')
    fig.suptitle('FASTGLIO 松耦合：有／无 GPS 欺骗的 ENU 时序误差', fontsize=18)
    fig.text(0.5, 0.01, '各次飞行分别与自身真值匹配；首点平移对齐，无旋转或尺度拟合；虚线为欺骗开始。',
             ha='center', fontsize=10)
    fig.tight_layout(rect=(0, .025, 1, .96))
    fig.savefig(args.output / 'enu-comparison.png', dpi=180)
    fig.savefig(args.output / 'enu-comparison.pdf')
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for axis, name in enumerate(('东向 E', '北向 N', '天向 U')):
        ax = axes[axis]
        ax.axhline(0, color='#adb5bd', linewidth=.8)
        for run, label, color in zip(runs, labels, ('#168aad', '#d6336c')):
            rows = run['errors']['/loose_odom']
            ax.plot(rows[:, 0], rows[:, axis+1], color=color, label=label, linewidth=1.7)
        ax.axvline(attack, color='#868e96', linestyle='--', linewidth=1)
        ax.set_ylabel(f'{name}误差 / m')
        ax.set_xlim(start, end)
        ax.grid(alpha=.25)
    axes[0].legend(loc='upper left')
    axes[-1].set_xlabel('相对 GPS 源启动的仿真时间 / s')
    fig.suptitle('只看 FASTGLIO 松耦合：有／无欺骗 ENU 误差', fontsize=17)
    fig.text(.5, .015, '第 20 秒开始向东注入 0.18 m/s 的 GPS 偏移；各次飞行以自身真值为参考。',
             ha='center', fontsize=10)
    fig.tight_layout(rect=(0, .035, 1, .96))
    fig.savefig(args.output / 'loose-enu-comparison.png', dpi=180)
    fig.savefig(args.output / 'loose-enu-comparison.pdf')
    plt.close(fig)

    # Keep the pure LiDAR/IMU baseline visible alongside both GNSS loose outputs.
    # The two LIO curves belong to separate flights; neither receives GNSS.
    for filename, title, include_loose in (
            ('lio-loose-enu-comparison', '纯 LiDAR/IMU 与 FASTGLIO 松耦合：ENU 误差对比', True),
            ('lidar-imu-enu-comparison', '纯 LiDAR/IMU 解算：两次飞行的 ENU 误差', False)):
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        for axis, name in enumerate(('东向 E', '北向 N', '天向 U')):
            ax = axes[axis]
            ax.axhline(0, color='#adb5bd', linewidth=.8)
            for run, label, color in zip(runs, labels, ('#168aad', '#d6336c')):
                rows = run['errors']['/Odometry']
                ax.plot(rows[:, 0], rows[:, axis+1], color=color,
                        label=f'纯 LiDAR/IMU（{label}飞行）',
                        linestyle='--' if include_loose else '-', linewidth=1.8)
                if include_loose:
                    rows = run['errors']['/loose_odom']
                    ax.plot(rows[:, 0], rows[:, axis+1], color=color,
                            label=f'FASTGLIO 松耦合（{label}）', linewidth=1.8)
            ax.axvline(attack, color='#868e96', linestyle=':', linewidth=1.2,
                       label='有欺骗组：GPS 欺骗开始' if axis == 0 else None)
            ax.set_ylabel(f'{name}误差 / m')
            ax.set_xlim(start, end)
            ax.grid(alpha=.25)
        handles, legend_labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, legend_labels, loc='upper center', ncol=2,
                   bbox_to_anchor=(.5, .947), fontsize=10)
        axes[-1].set_xlabel('相对 GPS 源启动的仿真时间 / s')
        fig.suptitle(title, fontsize=17, y=.99)
        fig.text(.5, .012,
                 '各次飞行以自身真值为参考，仅作首点平移对齐；纯 LiDAR/IMU 不接收 GPS，'
                 '两次飞行的数据不同。', ha='center', fontsize=9)
        fig.tight_layout(rect=(0, .035, 1, .85))
        fig.savefig(args.output / f'{filename}.png', dpi=180)
        fig.savefig(args.output / f'{filename}.pdf')
        plt.close(fig)

    fig = plt.figure(figsize=(14, 11))
    for index, run in enumerate(runs):
        ax3 = fig.add_subplot(2, 2, index+1, projection='3d')
        ax2 = fig.add_subplot(2, 2, index+3)
        for key, (label, color) in {'truth': ('Gazebo 真值', '#343a40'), **SERIES}.items():
            rows = run['aligned'][key]
            rows = rows[(rows[:, 0] >= start) & (rows[:, 0] <= end)]
            style = '--' if key.endswith('gnss_odom') else '-'
            ax3.plot(rows[:, 1], rows[:, 2], rows[:, 3], color=color, label=label, linestyle=style)
            ax2.plot(rows[:, 1], rows[:, 2], color=color, label=label, linestyle=style)
        ax3.set(xlabel='东 E / m', ylabel='北 N / m', zlabel='天 U / m', title=labels[index])
        ax3.view_init(elev=27, azim=-62)
        ax2.set(xlabel='东 E / m', ylabel='北 N / m', title=labels[index]+'：水平投影')
        ax2.axis('equal')
        ax2.grid(alpha=0.25)
        ax2.legend(fontsize=9)
    fig.suptitle('同航线两次闭环飞行：真值、LiDAR/IMU、松耦合与输入 GPS', fontsize=17)
    fig.tight_layout(rect=(0, .02, 1, .96))
    fig.savefig(args.output / 'trajectory-comparison.png', dpi=180)
    fig.savefig(args.output / 'trajectory-comparison.pdf')
    plt.close(fig)

    summary = dict(common_relative_interval_s=[start, end], attack_start_relative_s=attack,
                   mode=('offline_recompute' if all(run['replay_manifest'] for run in runs)
                         else 'closed_loop_capture'),
                   alignment='first matched point translation only; no rotation or scale fit',
                   truth_match_limit_ms=50,
                   sensor_profile_sha256=hashlib.sha256((args.clean / 'sensor_profile.json').read_bytes()).hexdigest(),
                   runs={})
    for label, run in zip(('clean', 'spoofed'), runs):
        result = dict(path=str(run['path']), metrics={},
                      replay_manifest=run['replay_manifest'],
                      flight_validation=run['flight_report'],
                      loose_validation=run['loose_report'],
                      final_injected_bias_enu_m=[run['logs'][-1]['attack_east_m'],
                                                run['logs'][-1]['attack_north_m'], 0.0])
        for topic, rows in run['errors'].items():
            common = rows[(rows[:, 0] >= start) & (rows[:, 0] <= end)]
            result['metrics'][topic] = dict(
                full=metrics(common), before_attack=metrics(common[common[:, 0] < attack]),
                after_attack=metrics(common[common[:, 0] >= attack]))
        summary['runs'][label] = result
    (args.output / 'comparison-summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    with (args.output / 'comparison-errors.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(('run', 'topic', 'relative_time_s', 'east_error_m', 'north_error_m', 'up_error_m'))
        for label, run in zip(('clean', 'spoofed'), runs):
            for topic, rows in run['errors'].items():
                for row in rows:
                    writer.writerow((label, topic, *row))

    fig, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(4)
    for index, (key, label, color) in enumerate(zip(('clean', 'spoofed'), labels, ('#168aad', '#d6336c'))):
        metric = summary['runs'][key]['metrics']['/loose_odom']['full']
        values = metric['rmse_enu_m']+[metric['rmse_3d_m']]
        bars = ax.bar(x+(index-.5)*.34, values, width=.34, label=label, color=color)
        ax.bar_label(bars, fmt='%.2f', padding=3, fontsize=10)
    ax.set_xticks(x, ('东 E', '北 N', '天 U', '三维'))
    ax.set(ylabel='RMSE / m', title='FASTGLIO 松耦合误差：同一相对时间区间')
    ax.legend()
    ax.margins(y=.18)
    fig.tight_layout()
    fig.savefig(args.output / 'rmse-comparison.png', dpi=180)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(12, 5.5))
    for index, (key, topic, label, color) in enumerate((
            ('clean', '/Odometry', '纯 LiDAR/IMU（无欺骗飞行）', '#74c0d8'),
            ('clean', '/loose_odom', '松耦合（无欺骗）', '#168aad'),
            ('spoofed', '/Odometry', '纯 LiDAR/IMU（有欺骗飞行）', '#f48fb1'),
            ('spoofed', '/loose_odom', '松耦合（有 GPS 欺骗）', '#d6336c'))):
        met = summary['runs'][key]['metrics'][topic]['full']
        values = met['rmse_enu_m']+[met['rmse_3d_m']]
        bars = ax.bar(x+(index-1.5)*.19, values, width=.18, label=label, color=color)
        ax.bar_label(bars, fmt='%.2f', padding=3, fontsize=9)
    ax.set_xticks(x, ('东 E', '北 N', '天 U', '三维'))
    ax.set(ylabel='RMSE / m', title='纯 LiDAR/IMU 与松耦合：同一相对时间区间的误差')
    ax.legend(ncol=2, fontsize=9)
    ax.margins(y=.3)
    fig.tight_layout()
    fig.savefig(args.output / 'lio-loose-rmse-comparison.png', dpi=180)
    fig.savefig(args.output / 'lio-loose-rmse-comparison.pdf')
    plt.close(fig)
    strides={run['replay_manifest']['point_filter_num'] for run in runs if run['replay_manifest']}
    sampling_text=(f'每 {next(iter(strides))} 点取一点' if len(strides)==1 else '各组记录的采样参数')
    introduction = ('对两次既有的 60 秒闭环飞行录包离线重算，统一使用 Velodyne 入口和'+sampling_text+'。'
                    '本次没有重新运行 PX4 飞行，真值与 GPS 注入记录来自原录包。'
                    if summary['mode']=='offline_recompute' else
                    '两次独立的 60 秒闭环航线飞行，使用相同传感器配置、航点和算法参数。')
    lines = ['# 有／无 GPS 欺骗的 FASTGLIO 松耦合对照', '',
             introduction+
             '有欺骗组在 GPS 源启动后 20 秒开始向东增加 0.18 m/s 的位置偏移。'
             '同一 GPS 位置同时进入 PX4 与 FASTGLIO，真实飞行路径因此也可能变化。', '',
             f'公共评估区间：GPS 源启动后 {start:.3f}～{end:.3f} 秒。'
             '各轨迹相对本次飞行真值进行首点平移对齐，不拟合旋转或尺度；真值时间匹配门限 50 ms。', '',
             '| 场景 | 解算 | 东 RMSE (m) | 北 RMSE (m) | 天 RMSE (m) | 三维 RMSE (m) |',
             '|---|---|---:|---:|---:|---:|']
    for key, label in zip(('clean', 'spoofed'), labels):
        result = summary['runs'][key]
        for topic in ('/loose_odom', '/Odometry'):
            met = result['metrics'][topic]['full']
            values = met['rmse_enu_m']+[met['rmse_3d_m']]
            lines.append(f'| {label} | {SERIES[topic][0]} | '+' | '.join(f'{v:.3f}' for v in values)+' |')
    lines += ['', '## 图表与数据', '',
              '![纯 LiDAR/IMU 与松耦合 ENU 时序误差](lio-loose-enu-comparison.png)', '',
              '![纯 LiDAR/IMU ENU 时序误差](lidar-imu-enu-comparison.png)', '',
              '![纯 LiDAR/IMU 与松耦合 RMSE](lio-loose-rmse-comparison.png)', '',
              '![松耦合 ENU 时序误差](loose-enu-comparison.png)', '',
              '![ENU 时序误差](enu-comparison.png)', '',
              '![三维轨迹及水平投影](trajectory-comparison.png)', '',
              '![松耦合 RMSE 对比](rmse-comparison.png)', '',
              '[汇总 JSON](comparison-summary.json) · [逐时刻误差 CSV](comparison-errors.csv)', '',
              '## 检查结果与解释限制', '',
              '无欺骗 GPS 直接来自 Gazebo 真值位置，当前未模拟 GNSS 接收机随机误差；'
              '松耦合使用演示配置的宽松残差门限，因此小的基线误差不代表真实设备定位精度，'
              '有欺骗的结果也不能作为抗欺骗性能结论。原始 LiDAR/IMU 前端未接收真值或 GNSS 反馈。', '']
    lines += ['图中的“纯 LiDAR/IMU（有 GPS 欺骗飞行）”表示从受骗飞行中记录的雷达和 IMU '
              '进行解算，并不表示纯 LIO 直接融合了假 GPS。两次飞行的实际轨迹、采样和消息时序不同，'
              '欺骗前曲线也存在差异，不能把两条纯 LIO 曲线之差全部归因于 GPS 欺骗。', '']
    for key, label in zip(('clean', 'spoofed'), labels):
        result = summary['runs'][key]
        flight, loose = result['flight_validation'], result['loose_validation']
        sync = flight.get('lidar_alignment') or {}
        lines.append(f"- {label}：{'原飞行验证（继承）' if result['replay_manifest'] else '飞行验证'}，已解锁={flight['armed']}，OFFBOARD={flight['offboard']}，"
                     f"PX4 注入坐标匹配率={flight['injection_match_fraction']:.1%}，"
                     f"FASTGLIO 接受更新={loose['gnss_updates_accepted']}；"
                     f"在线 LiDAR/GPS 200 ms 配对通过率={sync.get('lidar_sync_fraction', 0):.1%}，"
                     f"完整飞行及同步门禁通过={flight['pass_flight_gate']}。")
    lines += ['', '当前更密的雷达点云存在传输／处理延迟，在线配对时间差可能超过 200 ms；'
              '超限检查仍保留，未为本次图表放宽门限。图表使用各传感器／里程计原始时间戳，'
              '完整检查记录保存在两组运行目录中。', '']
    if summary['mode']=='offline_recompute':
        lines += ['本次重算通过 rosbag 仿真时钟按原传感器时间戳输入；'
                  '上述 200 ms 在线配对门禁是原飞行采集时的记录，不是本次重算重新测得的在线延迟。'
                  '新的解算验证见各组 `fastglio-live-report.json`，来源和配置见 `replay-manifest.json`。', '']
    (args.output / 'REPORT.md').write_text('\n'.join(lines))
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
