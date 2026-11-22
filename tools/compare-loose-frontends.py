#!/usr/bin/env python3
"""Compare the original online run and explicit replay frontend settings."""
import argparse
import importlib.util
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

spec=importlib.util.spec_from_file_location('compare',Path(__file__).with_name('compare-fastglio-spoof-runs.py'))
compare=importlib.util.module_from_spec(spec);spec.loader.exec_module(compare)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--original',type=Path,required=True)
    parser.add_argument('--half',type=Path,required=True)
    parser.add_argument('--quarter',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    groups={label:{scenario:compare.read_run(path/scenario) for scenario in ('clean','spoofed')}
            for label,path in [('MARSIM 原在线',args.original),('Velodyne 取 1/2',args.half),('Velodyne 取 1/4',args.quarter)]}
    start=max(rows[0,0] for group in groups.values() for run in group.values() for rows in run['errors'].values())
    end=min(rows[-1,0] for group in groups.values() for run in group.values() for rows in run['errors'].values())
    results={}
    for label,group in groups.items():
        results[label]={}
        for scenario,run in group.items():
            results[label][scenario]={}
            for topic in ('/Odometry','/loose_odom'):
                rows=run['errors'][topic];rows=rows[(rows[:,0]>=start)&(rows[:,0]<=end)].copy()
                rows[:,1:]-=rows[0,1:]
                results[label][scenario][topic]=compare.metrics(rows)
    summary=dict(common_window_s=[start,end],method='same relative interval and first matched point translation; no rotation or scale fit',
                 note='original online outputs and offline replay differ in estimator startup/buffering as well as frontend; not a strict one-factor causal ablation',results=results)
    (args.output/'frontend-summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    plt.rcParams.update({'font.family':'Noto Sans CJK SC','font.size':11})
    fig,axes=plt.subplots(2,1,figsize=(11,8))
    x=np.arange(len(results))
    for ax,topic,title in zip(axes,('/Odometry','/loose_odom'),('纯 LiDAR/IMU','LiDAR + IMU + GPS 松耦合')):
        for i,(scenario,label,color) in enumerate([('clean','无欺骗','#168aad'),('spoofed','有 GPS 欺骗','#d6336c')]):
            vals=[group[scenario][topic]['rmse_3d_m'] for group in results.values()]
            bars=ax.bar(x+(i-.5)*.32,vals,width=.3,label=label,color=color)
            ax.bar_label(bars,fmt='%.3f',padding=3)
        ax.set_xticks(x,list(results));ax.set_ylabel('三维 RMSE / m');ax.set_title(title)
        ax.legend();ax.margins(y=.22);ax.grid(axis='y',alpha=.2)
    fig.suptitle('同两份飞行录包：原在线结果与 Velodyne 前端重算',fontsize=16)
    fig.tight_layout(rect=(0,.025,1,.95))
    fig.savefig(args.output/'frontend-results.png',dpi=180)
    fig.savefig(args.output/'frontend-results.pdf')
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
