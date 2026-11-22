#!/usr/bin/env python3
"""Export four large-font PNGs from a matched clean/spoofed flight pair."""
import argparse
import csv
import hashlib
import importlib.util
import json
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

SERIES = [
    ('truth', '真实轨迹', '#343A40', ':', 2.7),
    ('/msim/spoofed_gnss_odom', 'GPS', '#29945C', '--', 2.1),
    ('/Odometry', 'LiDAR / IMU', '#2476B9', '-', 2.8),
    ('/loose_odom', 'LiDAR / IMU / GPS 松组合', '#E77724', '-', 2.7),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--clean', required=True, type=Path)
    parser.add_argument('--spoofed', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    runs = {name: comparison.read_run(path.resolve()) for name, path in
            [('clean', args.clean), ('spoofed', args.spoofed)]}
    if runs['clean']['attack'] is not None or runs['spoofed']['attack'] is None:
        raise ValueError('the pair does not contain one clean and one spoofed run')
    hashes = {}
    for filename in ('sensor_profile.json', 'route_plan.json', 'algorithm.yaml', 'vehicle.sdf', 'world.sdf'):
        contents = [(run['path']/filename).read_bytes() for run in runs.values()]
        if contents[0] != contents[1]:
            raise ValueError('comparison inputs differ: '+filename)
        hashes[filename] = hashlib.sha256(contents[0]).hexdigest()
    settings = [json.loads((run['path']/'capture-settings.json').read_text()) for run in runs.values()]
    for field in ('duration_s', 'route_clock', 'flight_pattern', 'frontend', 'point_filter_num'):
        if settings[0][field] != settings[1][field]:
            raise ValueError('capture settings differ: '+field)
    all_rows = [rows for run in runs.values() for rows in
                [*run['aligned'].values(), *run['errors'].values()]]
    start = max(rows[0, 0] for rows in all_rows)
    end = min(rows[-1, 0] for rows in all_rows)
    if end <= start:
        raise ValueError('empty common time interval')
    tracks, errors = {}, {}
    for name, run in runs.items():
        tracks[name] = {topic: rows[(rows[:, 0] >= start) & (rows[:, 0] <= end)]
                        for topic, rows in run['aligned'].items()}
        errors[name] = {topic: rows[(rows[:, 0] >= start) & (rows[:, 0] <= end)]
                        for topic, rows in run['errors'].items()}
    xyz = np.vstack([rows[:, 1:] for group in tracks.values() for rows in group.values()])
    low, high = xyz.min(0), xyz.max(0)
    margin = np.maximum((high-low)*.065, .6)
    low -= margin
    high += margin
    err = np.vstack([rows[:, 1:] for group in errors.values() for rows in group.values()])
    err_low, err_high = err.min(0), err.max(0)
    err_margin = np.maximum((err_high-err_low)*.12, .1)
    err_low -= err_margin
    err_high += err_margin
    plt.rcParams.update({
        'font.family': 'Noto Sans CJK SC', 'font.size': 19,
        'text.color': '#30343B', 'axes.labelcolor': '#30343B',
        'xtick.color': '#4A525B', 'ytick.color': '#4A525B',
        'axes.spines.top': False, 'axes.spines.right': False,
        'axes.unicode_minus': False,
    })
    summary = dict(common_interval_s=[start, end],input_sha256=hashes,
                   same_axis_limits_across_cases=True,
                   method='each estimate uses its first truth-matched translation; no rotation/scale fit; no smoothing',
                   injection_marker_shown=False,axis_scaling_3d='equal meters in ENU',runs={})
    for name, run in runs.items():
        fig = plt.figure(figsize=(9.2, 8.6), facecolor='white')
        ax = fig.add_subplot(projection='3d', computed_zorder=False)
        handles = {}
        for topic, label, color, style, width in reversed(SERIES):
            rows = tracks[name][topic]
            handles[topic], = ax.plot(*rows[:, 1:].T, color=color, linestyle=style,
                                     linewidth=width, label=label, zorder=3)
        ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]))
        ax.set_box_aspect(high-low, zoom=1.18)
        ax.view_init(elev=24, azim=-58)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.set_major_locator(MaxNLocator(nbins=4))
            axis.pane.fill = False
            axis.pane.set_edgecolor('#E4E8EC')
            axis._axinfo['grid'].update(color='#DFE5EA', linewidth=.7)
            axis.line.set_color('#A0A8B0')
        ax.tick_params(labelsize=16, pad=4)
        ax.set_xlabel('东 E / m', fontsize=20, labelpad=19)
        ax.set_ylabel('北 N / m', fontsize=20, labelpad=34)
        ax.set_zlabel('天 U / m', fontsize=20, labelpad=16)
        fig.legend([handles[topic] for topic, *_ in SERIES],
                   [label for _, label, *_ in SERIES],
                   loc='upper center', bbox_to_anchor=(.5, 1.01), ncol=2,
                   frameon=False, fontsize=18, handlelength=2.1, columnspacing=1.4)
        fig.subplots_adjust(left=0, right=1, bottom=.06, top=.94)
        fig.savefig(output/(name+'-3d-compact.png'), dpi=300, facecolor='white',
                    bbox_inches='tight', pad_inches=.06)
        plt.close(fig)

        fig, axes = plt.subplots(3, 1, figsize=(12.5, 9.8), sharex=True, facecolor='white')
        legend_handles = {}
        for index, axis_name in enumerate(('东 E', '北 N', '天 U')):
            ax = axes[index]
            ax.axhline(0, color='#89939D', linewidth=.8, zorder=1)
            for topic, label, color, style, width in reversed(SERIES[1:]):
                rows = errors[name][topic]
                line, = ax.plot(rows[:, 0], rows[:, index+1], color=color,
                                linestyle=style, linewidth=width, label=label, zorder=3)
                legend_handles[topic] = line
            ax.set_ylabel(axis_name+' 误差 / m', fontsize=20, labelpad=12)
            ax.set_ylim(err_low[index], err_high[index])
            ax.set_xlim(start, end)
            ax.grid(color='#E3E7EC', linewidth=.8)
            ax.yaxis.set_major_locator(MaxNLocator(nbins=4))
            ax.tick_params(labelsize=17)
            ax.spines['left'].set_color('#A0A8B0')
            ax.spines['bottom'].set_color('#A0A8B0')
        axes[-1].set_xlabel('仿真时间 / s', fontsize=20, labelpad=10)
        fig.legend([legend_handles[topic] for topic, *_ in SERIES[1:]],
                   [label for _, label, *_ in SERIES[1:]], loc='upper center',
                   bbox_to_anchor=(.5, .995), ncol=3, frameon=False,
                   fontsize=17, columnspacing=1.3, handlelength=2.1)
        fig.subplots_adjust(left=.14, right=.98, bottom=.10, top=.91, hspace=.20)
        fig.savefig(output/(name+'-enu-error.png'), dpi=300, facecolor='white')
        plt.close(fig)
        for kind, group in [('trajectories', tracks[name]), ('errors', errors[name])]:
            with (output/(name+'-'+kind+'.csv')).open('w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(('series', 'time_since_gps_source_s', 'east_m', 'north_m', 'up_m'))
                for topic, rows in group.items():
                    writer.writerows((topic, *row) for row in rows)
        summary['runs'][name] = dict(source=str(run['path']),settings=json.loads((run['path']/'capture-settings.json').read_text()),
                                   attack_start_relative_s=run['attack'],
                                   error_metrics={topic: comparison.metrics(rows) for topic, rows in errors[name].items()})
    (output/'figure-sources.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(output)


if __name__ == '__main__':
    main()
