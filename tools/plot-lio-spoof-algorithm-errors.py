#!/usr/bin/env python3
"""Plot pure frontend ENU errors for matched clean/spoofed source flights."""
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
import rosbag

spec = importlib.util.spec_from_file_location('enu', Path(__file__).with_name('plot-fastglio-enu-error.py'))
enu = importlib.util.module_from_spec(spec)
spec.loader.exec_module(enu)

ALGORITHMS = [
    ('fast-lio-velodyne', 'FAST-LIO', '/Odometry', '#2476B9', '-'),
    ('lio-sam-patched', 'LIO-SAM', '/lio_sam/mapping/odometry', '#E77724', '-'),
    ('ligo', 'LIGO', '/Odometry', '#29945C', '--'),
    ('glio-front-end', 'GLIO 前端', '/odom', '#9A529D', '-.'),
]


def load_case(path):
    log = [json.loads(line) for line in (path/'injected-gps.jsonl').read_text().splitlines()]
    origin = log[0]['sim_time_ns']/1e9
    truth = []
    with rosbag.Bag(str(path/'sensors.bag')) as bag:
        for _, msg, _ in bag.read_messages(topics=['/ground_truth/state']):
            truth.append((enu.stamp(msg), msg))
    truth.sort(key=lambda row: row[0])
    truth_times = [row[0] for row in truth]
    data, reports = {}, {}
    for algorithm, label, topic, *_ in ALGORITHMS:
        root = path/'comparisons'/algorithm
        track = []
        with rosbag.Bag(str(root/'algorithm.bag')) as bag:
            for _, msg, _ in bag.read_messages(topics=[topic]):
                track.append((enu.stamp(msg), enu.position(msg)))
        track.sort(key=lambda row: row[0])
        finite = [(t, p) for t, p in track if np.isfinite(p).all()]
        rows = np.array(enu.errors_for(finite, truth, truth_times), dtype=float)
        if len(rows) < 10:
            raise ValueError(f'too few valid truth matches: {path.name}/{algorithm}')
        rows[:, 0] -= origin
        data[algorithm] = rows
        reports[algorithm] = dict(label=label, odometry_messages=len(track),
                                  finite_messages=len(finite), truth_matched=len(rows),
                                  relative_interval_s=[rows[0, 0], rows[-1, 0]],
                                  max_output_gap_s=float(np.max(np.diff(rows[:, 0]))),
                                  rmse_enu_m=np.sqrt(np.mean(rows[:, 1:]**2, axis=0)).tolist(),
                                  rmse_3d_m=float(np.sqrt(np.mean(np.sum(rows[:, 1:]**2, axis=1)))))
    return data, reports, (truth[0][0]-origin, truth[-1][0]-origin)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--group', required=True, type=Path)
    parser.add_argument('--yscale', choices=('linear', 'symlog'), default='linear')
    parser.add_argument('--linthresh', type=float, default=.5)
    args = parser.parse_args()
    if args.linthresh <= 0:
        parser.error('--linthresh must be positive')
    suffix = '-symlog' if args.yscale == 'symlog' else ''
    group = args.group.resolve()
    output = group/'figures'
    output.mkdir(exist_ok=True)
    cases, reports, intervals = {}, {}, {}
    for name in ('clean', 'spoofed'):
        cases[name], reports[name], intervals[name] = load_case(group/name)
    start = max(interval[0] for interval in intervals.values())
    end = min(interval[1] for interval in intervals.values())
    # Use the same limits; retain each frontend's actual coverage and gaps.
    all_errors = np.vstack([rows[(rows[:, 0]>=start)&(rows[:, 0]<=end), 1:]
                            for case in cases.values() for rows in case.values()])
    low, high = all_errors.min(0), all_errors.max(0)
    padding = np.maximum((high-low)*.10, .15)
    low -= padding
    high += padding
    plt.rcParams.update({'font.family':'Noto Sans CJK SC', 'font.size':18,
                         'axes.spines.top':False, 'axes.spines.right':False,
                         'axes.labelcolor':'#30343B', 'text.color':'#30343B',
                         'xtick.color':'#4A525B', 'ytick.color':'#4A525B',
                         'axes.unicode_minus':False})
    for case, series in cases.items():
        fig, axes = plt.subplots(3, 1, figsize=(12.5, 9.8), sharex=True, facecolor='white')
        handles = []
        for row, label in enumerate(('东 E', '北 N', '天 U')):
            ax = axes[row]
            ax.axhline(0, color='#89939D', lw=.8)
            for algorithm, name, _, color, style in ALGORITHMS:
                values = series[algorithm]
                values = values[(values[:, 0]>=start)&(values[:, 0]<=end)]
                # Missing output is a visible break, never an interpolated path.
                gap = np.flatnonzero(np.diff(values[:, 0])>1.)+1
                draw = values.copy()
                for index in gap[::-1]:
                    draw = np.insert(draw, index, np.full((1, 4), np.nan), axis=0)
                line, = ax.plot(draw[:, 0], draw[:, row+1], color=color, ls=style,
                                lw=2.3, label=name)
                if row == 0:
                    handles.append(line)
            ax.set_ylabel(label+' 误差 / m', fontsize=20, labelpad=12)
            if args.yscale == 'symlog':
                ax.set_yscale('symlog', linthresh=args.linthresh, linscale=.5, base=10)
            ax.set_ylim(low[row], high[row])
            ax.set_xlim(start, end)
            ax.grid(color='#E3E7EC', lw=.8)
            if args.yscale == 'symlog':
                ax.set_yticks(([-.5, 0, .5, 2, 8],
                               [-50, -5, 0, 5, 50, 200],
                               [-5, -1, 0, 1, 5, 20])[row])
                ax.set_yticklabels((["-0.5", "0", "0.5", "2", "8"],
                                    ["-50", "-5", "0", "5", "50", "200"],
                                    ["-5", "-1", "0", "1", "5", "20"])[row])
                ax.minorticks_off()
            else:
                ax.yaxis.set_major_locator(MaxNLocator(nbins=4))
            ax.tick_params(labelsize=17)
            ax.spines['left'].set_color('#A0A8B0')
            ax.spines['bottom'].set_color('#A0A8B0')
        # Preserve LIGO's full drift while making the small north errors readable.
        if args.yscale == 'linear' and high[1]-low[1] > 40:
            zoom = axes[1].inset_axes([.045, .48, .34, .47])
            for algorithm, _, _, color, style in ALGORITHMS:
                values = series[algorithm].copy()
                for index in (np.flatnonzero(np.diff(values[:, 0])>1.)+1)[::-1]:
                    values=np.insert(values,index,np.full((1,4),np.nan),axis=0)
                zoom.plot(values[:, 0], values[:, 2], color=color, ls=style, lw=1.6)
            zoom.set(xlim=(start, end), ylim=(-5, 2))
            zoom.set_xticks([20, 40, 60])
            zoom.set_yticks([-4, 0])
            zoom.tick_params(labelsize=11, pad=1)
            zoom.set_title('北向局部放大', fontsize=12, loc='left', pad=3)
            zoom.grid(color='#E3E7EC', lw=.6)
            for spine in zoom.spines.values():
                spine.set_color('#A0A8B0')
        axes[-1].set_xlabel('仿真时间 / s', fontsize=20, labelpad=10)
        fig.legend(handles, [row[1] for row in ALGORITHMS], loc='upper center',
                   bbox_to_anchor=(.5,.995), ncol=4, fontsize=18,
                   frameon=False, columnspacing=1.2, handlelength=2.1)
        fig.subplots_adjust(left=.14,right=.98,bottom=.13 if suffix else .10,top=.91,hspace=.20)
        if suffix:
            fig.text(.56,.015,'纵轴：对称对数刻度，零附近线性',
                     fontsize=12,color='#646D76',ha='center')
        fig.savefig(output/(case+'-lio-algorithms-enu'+suffix+'.png'),dpi=300,facecolor='white')
        plt.close(fig)
        with (output/(case+'-errors.csv')).open('w',newline='') as file:
            writer=csv.writer(file)
            writer.writerow(('algorithm','time_since_gps_source_s','east_error_m','north_error_m','up_error_m'))
            for algorithm, rows in series.items():
                writer.writerows((algorithm,*row) for row in rows)
    common_start=max(rows[0,0] for series in cases.values() for rows in series.values())
    common_end=min(rows[-1,0] for series in cases.values() for rows in series.values())
    for case,series in cases.items():
        for algorithm,rows in series.items():
            common=rows[(rows[:,0]>=common_start)&(rows[:,0]<=common_end),1:]
            reports[case][algorithm]['common_window_rmse_enu_m']=np.sqrt(np.mean(common**2,axis=0)).tolist()
            reports[case][algorithm]['common_window_rmse_3d_m']=float(np.sqrt(np.mean(np.sum(common**2,axis=1))))
    summary=dict(plot_interval_s=[start,end],common_valid_interval_s=[common_start,common_end],same_axis_limits_across_cases=True,
                 vertical_scale=args.yscale,linear_region_m=args.linthresh if suffix else None,
                 symlog_linscale=.5 if suffix else None,
                 alignment='first truth-matched translation per algorithm; no rotation or scale fit',
                 truth_match_limit_ms=50,results=reports,
                 benchmark=json.loads((group/'benchmark-manifest.json').read_text()))
    (output/('summary'+suffix+'.json')).write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(reports,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
