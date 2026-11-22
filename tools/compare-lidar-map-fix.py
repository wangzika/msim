#!/usr/bin/env python3
"""Compare fresh online flights before/after the local-map parameter fix."""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--before', type=Path, required=True)
    ap.add_argument('--after', type=Path, required=True)
    a = ap.parse_args()
    spec = importlib.util.spec_from_file_location('lidar_comparison',
             Path(__file__).with_name('plot-lidar-comparison.py'))
    helper = importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    groups = {stage: {name: helper.read_run(root/name) for name, *_ in helper.MODELS}
              for stage, root in [('before', a.before.resolve()), ('after', a.after.resolve())]}
    helper.validate_inputs({stage+'/'+name: run for stage, runs in groups.items()
                            for name, run in runs.items()})
    roots = {}
    for name, *_ in helper.MODELS:
        before, after = [groups[s][name] for s in ('before', 'after')]
        assert before['profile'] == after['profile'], 'Sensor change: '+name
        assert before['algorithm'] == after['algorithm'], 'Algorithm YAML change: '+name
        roots[name] = {}
        for stage, run in [('before', before), ('after', after)]:
            p = yaml.safe_load((run['path']/'parameters.yaml').read_text())
            roots[name][stage] = dict(global_cube_side_length=p.get('cube_side_length'),
                                      effective_cube_side_length=p.get('cube_side_length', 200.),
                                      algorithm_det_range=p['mapping']['det_range'])
        assert roots[name]['before']['global_cube_side_length'] is None
        assert roots[name]['after']['effective_cube_side_length'] == 1000.
        assert after['flight']['pass_flight_gate']
        assert after['flight']['pass_lidar_sync_gate']
        assert after['fusion']['pass_live_loose_gate']
    all_data = [d for runs in groups.values() for run in runs.values() for d in run['errors'].values()]
    lo = math.ceil(max(d[0, 0] for d in all_data)*10)/10
    hi = math.floor(min(d[-1, 0] for d in all_data)*10)/10
    assert hi-lo >= 30
    output = a.after.resolve()/'figures';output.mkdir(exist_ok=True)
    summary = dict(common_sim_interval_s=[lo, hi], map_parameters=roots,
                   source_groups=dict(before=str(a.before.resolve()), after=str(a.after.resolve())),
                   input_hashes={stage: {name: {f: helper.sha(run['path']/f)
                       for f in ('world.sdf', 'route_plan.json', 'sensor_profile.json',
                                 'algorithm.yaml', 'algorithm.launch', 'parameters.yaml', 'sensor-plugins.json')}
                       for name, run in runs.items()} for stage, runs in groups.items()},
                   method='truth interpolated at measurement stamp; first matched translation only; no rotation/scale fit; no smoothing',
                   caveat='separate closed-loop online flights with identical scene, sensor presets and route; not same-bag paired replay or repeated-trial statistics',
                   runs={})
    with (output/'before-after-metrics.csv').open('w', newline='') as stream:
        w = csv.writer(stream)
        w.writerow(['lidar', 'solution', 'stage', 'samples', 'east_rmse_m', 'north_rmse_m',
                    'up_rmse_m', 'three_d_rmse_m', 'horizontal_rmse_m', 'p95_3d_m', 'max_3d_m'])
        for name, *_ in helper.MODELS:
            summary['runs'][name] = {}
            for topic, _, stem in helper.SOLUTIONS:
                m = {}
                for stage in ('before', 'after'):
                    d = groups[stage][name]['errors'][topic]
                    d = d[(d[:, 0] >= lo) & (d[:, 0] <= hi)]
                    m[stage] = helper.metrics(d)
                    w.writerow([name, stem, stage, m[stage]['samples'], *m[stage]['rmse_enu_m'],
                                m[stage]['rmse_3d_m'], m[stage]['horizontal_rmse_m'],
                                m[stage]['p95_3d_error_m'], m[stage]['max_3d_error_m']])
                summary['runs'][name][stem] = m
    plt.rcParams.update({'font.family': 'Noto Sans CJK SC', 'axes.unicode_minus': False})
    for topic, title, stem in helper.SOLUTIONS:
        fig, axes = plt.subplots(3, 4, figsize=(23, 10), sharex=True)
        for col, (name, label, _, _) in enumerate(helper.MODELS):
            for row, axis in enumerate(('东 E', '北 N', '天 U')):
                ax = axes[row, col]
                for stage, text, color, style in [('before', '修正前', '#B45468', '--'),
                                                 ('after', '修正后', '#167D9C', '-')]:
                    d = groups[stage][name]['errors'][topic]
                    d = d[(d[:, 0] >= lo) & (d[:, 0] <= hi)]
                    ax.plot(d[:, 0], d[:, row+1], label=text, color=color, ls=style, lw=2)
                ax.axhline(0, color='#93999E', lw=.7)
                ax.grid(color='#E1E5E9');ax.tick_params(labelsize=16)
                ax.spines[['top', 'right']].set_visible(False)
                ax.yaxis.set_major_locator(MaxNLocator(nbins=4))
                ax.set_ylabel(axis+' 误差 / m', fontsize=19)
                ax.set_xlim(lo, hi)
            axes[0, col].set_title(label, fontsize=23, pad=12)
            axes[-1, col].set_xlabel('仿真时间 / s', fontsize=20)
        handles, labels = axes[0, 0].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(.5, .96),
                   ncol=2, fontsize=21, frameon=False)
        fig.suptitle('无欺骗 · '+title+' · 在线重跑', fontsize=24, y=.998)
        fig.subplots_adjust(left=.06, right=.99, bottom=.09, top=.85, hspace=.25, wspace=.36)
        fig.savefig(output/(stem+'-before-after.png'), dpi=220, facecolor='white');plt.close(fig)
    (output/'before-after-summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print('Common interval:', lo, hi)
    for name, *_ in helper.MODELS:
        for _, _, stem in helper.SOLUTIONS:
            m = summary['runs'][name][stem]
            print(name, stem, 'before ENU', m['before']['rmse_enu_m'],
                  'after ENU', m['after']['rmse_enu_m'], 'after 3D', m['after']['rmse_3d_m'])


if __name__ == '__main__':
    main()
