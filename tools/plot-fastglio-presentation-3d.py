#!/usr/bin/env python3
"""Export ground truth, GPS, LIO and loose-fusion trajectories from one flight."""
import argparse
import csv
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
    ('truth', '真实轨迹', '#343A40', ':', 2.5),
    ('/msim/spoofed_gnss_odom', 'GPS', '#29945C', '--', 2.0),
    ('/Odometry', 'LiDAR / IMU', '#2476B9', '-', 2.8),
    ('/loose_odom', 'LiDAR / IMU / GPS 松组合', '#E77724', '-', 2.5),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    run = comparison.read_run(args.run.resolve())
    start = max(run['aligned'][topic][0, 0] for topic, *_ in SERIES)
    end = min(run['aligned'][topic][-1, 0] for topic, *_ in SERIES)
    if end <= start:
        raise ValueError('no common recorded interval')
    tracks = {}
    for topic, *_ in SERIES:
        rows = run['aligned'][topic]
        tracks[topic] = rows[(rows[:, 0] >= start) & (rows[:, 0] <= end)]
    xyz = np.vstack([rows[:, 1:] for rows in tracks.values()])
    low, high = xyz.min(0), xyz.max(0)
    margin = np.maximum((high-low)*.06, .5)
    low -= margin
    high += margin
    plt.rcParams.update({
        'font.family': 'Noto Sans CJK SC', 'font.size': 13,
        'axes.labelcolor': '#333333', 'text.color': '#333333',
        'xtick.color': '#555555', 'ytick.color': '#555555',
        'svg.fonttype': 'none',
    })
    fig = plt.figure(figsize=(11, 8.4), facecolor='white')
    ax = fig.add_subplot(projection='3d', computed_zorder=False)
    # Draw the dashed/dotted references last where the trajectories overlap.
    handles = {}
    for topic, label, color, style, width in reversed(SERIES):
        rows = tracks[topic]
        handles[topic], = ax.plot(*rows[:, 1:].T, color=color, linestyle=style,
                                 linewidth=width, label=label, zorder=3)
    ax.set(xlabel='东 E / m', ylabel='北 N / m', zlabel='天 U / m',
           xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]))
    ax.set_box_aspect(high-low)
    ax.view_init(elev=24, azim=-58)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_major_locator(MaxNLocator(nbins=5))
        axis.pane.fill = False
        axis.pane.set_edgecolor('#E4E8EC')
        axis._axinfo['grid'].update(color='#DFE5EA', linewidth=.65)
        axis.line.set_color('#A0A8B0')
    ax.tick_params(labelsize=11, pad=2)
    ax.set_xlabel('东 E / m', labelpad=11)
    ax.set_ylabel('北 N / m', labelpad=11)
    ax.set_zlabel('天 U / m', labelpad=9)
    fig.legend([handles[topic] for topic, *_ in SERIES],
               [label for _, label, *_ in SERIES],
               loc='upper center', bbox_to_anchor=(.5, .98), ncol=2,
               frameon=False, fontsize=13, handlelength=2.4, columnspacing=1.7)
    fig.subplots_adjust(left=.01, right=.96, bottom=.04, top=.87)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    stem = output/'gps-lio-loose-truth-3d'
    for extension in ('png', 'svg'):
        fig.savefig(stem.with_suffix('.'+extension), dpi=300, facecolor='white')
    plt.close(fig)
    with stem.with_suffix('.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(('series', 'time_since_gps_source_s', 'east_m', 'north_m', 'up_m'))
        for topic, label, *_ in SERIES:
            writer.writerows((label, *row) for row in tracks[topic])
    metadata = dict(source_run=str(run['path']), injection_marker_shown=False,
                    ground_truth_topic='/ground_truth/state',
                    common_interval_s=[start, end],
                    coordinates='ENU; each estimate uses first truth-matched translation, no rotation/scale fit',
                    axis_scaling='equal meters in E/N/U',
                    source_configuration=json.loads((run['path']/'capture-settings.json').read_text()),
                    series={topic: dict(label=label, samples=len(tracks[topic]))
                            for topic, label, *_ in SERIES})
    stem.with_suffix('.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    print(stem.with_suffix('.png'))


if __name__ == '__main__':
    main()
