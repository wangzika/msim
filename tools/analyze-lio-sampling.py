#!/usr/bin/env python3
"""Summarize controlled LIO sampling trials and their matching geometry."""
import argparse
from bisect import bisect_left
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag


def load_bag(path):
    odom, truth = [], []
    with rosbag.Bag(str(path)) as bag:
        for topic, msg, _ in bag.read_messages(topics=['/Odometry', '/ground_truth/state']):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            row = (msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            (odom if topic == '/Odometry' else truth).append(row)
    return np.array(odom), np.array(truth)


def errors(odom, truth, start, end):
    rows = []
    for row in odom:
        t = row[0]
        if not start <= t <= end:
            continue
        index = np.searchsorted(truth[:, 0], t)
        candidates = [i for i in (index-1, index) if 0 <= i < len(truth)]
        near = min(candidates, key=lambda i: abs(truth[i, 0]-t))
        if abs(truth[near, 0]-t) > .05:
            continue
        q, refq = row[4:], truth[near, 4:]
        norm = np.linalg.norm(q)*np.linalg.norm(refq)
        angle = np.degrees(2*np.arccos(np.clip(abs(np.dot(q, refq))/norm, 0, 1)))
        rows.append((t, *(row[1:4]-truth[near, 1:4]), angle))
    result = np.array(rows)
    result[:, 1:4] -= result[0, 1:4]
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--group', type=Path, required=True)
    parser.add_argument('--start', type=float)
    parser.add_argument('--end', type=float)
    args = parser.parse_args()
    cases = []
    for path in sorted(args.group.iterdir()):
        if not (path/'summary.json').is_file() or (path/'INVALID.json').exists():
            continue
        case = json.loads((path/'experiment.json').read_text())
        odom, truth = load_bag(path/'algorithm.bag')
        if np.any(np.diff(odom[:,0])<=0):
            raise ValueError(f'non-increasing odometry time in {path}')
        diag = np.genfromtxt(path/'matching.csv', delimiter=',', names=True)
        if np.any(np.diff(diag['stamp_s'])<=0):
            raise ValueError(f'non-increasing diagnostic time in {path}')
        cases.append(dict(path=path, config=case, odom=odom, truth=truth, diag=diag))
    if not cases:
        raise ValueError('no completed cases')
    start = args.start if args.start is not None else max(case['odom'][0, 0] for case in cases)
    end = args.end if args.end is not None else min(case['odom'][-1, 0] for case in cases)
    start, end = start+1e-7, end-1e-7
    results = []
    for case in cases:
        rows = errors(case['odom'], case['truth'], start, end)
        diag = case['diag']
        diag = diag[(diag['stamp_s'] >= start) & (diag['stamp_s'] <= end)]
        effective = np.maximum(diag['effective_points'], 1)
        diagonal = np.column_stack([diag['info_'+axis] for axis in 'ENU'])
        result = dict(**case['config'], matched_frames=len(rows), diagnostic_frames=len(diag),
                      rmse_enu_m=np.sqrt(np.mean(rows[:, 1:4]**2, axis=0)).tolist(),
                      rmse_3d_m=float(np.sqrt(np.mean(np.sum(rows[:, 1:4]**2, axis=1)))),
                      final_enu_error_m=rows[-1, 1:4].tolist(),
                      attitude_error_median_deg=float(np.median(rows[:, 4])),
                      attitude_error_p95_deg=float(np.percentile(rows[:, 4], 95)),
                      points_prevoxel_mean=float(np.mean(diag['prevoxel_points'])),
                      points_voxel_mean=float(np.mean(diag['voxel_points'])),
                      effective_points_mean=float(np.mean(diag['effective_points'])),
                      normal_axis_fraction_mean=[float(np.mean(diag['normal_'+a]/effective)) for a in 'ENU'],
                      normal_axis_count_mean=[float(np.mean(diag['normal_'+a])) for a in 'ENU'],
                      mean_abs_residual_m=float(np.mean(diag['mean_abs_residual'])),
                      normal_gram_diag_mean=np.mean(diagonal, axis=0).tolist(),
                      normalized_normal_gram_diag_mean=np.mean(diagonal/effective[:, None], axis=0).tolist(),
                      normalized_eigenvalues_median=[float(np.median(diag[k]/effective))
                                                    for k in ('lambda_min','lambda_mid','lambda_max')],
                      normal_gram_condition_median=float(np.median(diag['lambda_max']/np.maximum(diag['lambda_min'],1e-12))),
                      weak_axis_abs_mean=[float(np.mean(abs(diag['weak_'+a]))) for a in 'ENU'],
                      solve_ms_median=float(np.median(diag['solve_ms'])),
                      solve_ms_p95=float(np.percentile(diag['solve_ms'],95)))
        for field in ('line_neighborhood_fraction', 'line_neighborhood_U_fraction',
                      'middle_largest_ratio_median','ls_pca_normal_angle_median_deg'):
            if field in diag.dtype.names:
                result[field+'_mean']=float(np.mean(diag[field]))
        result['final_accel_bias_mps2']=[float(diag['accel_bias_'+a][-1]) for a in 'xyz']
        result['final_gyro_bias_radps']=[float(diag['gyro_bias_'+a][-1]) for a in 'xyz']
        results.append(result)
        case['error_rows'] = rows
    report = dict(common_window_s=[start,end], source='same fixed annotated bag; pure LIO; no GNSS',
                  alignment='first matched point translation only; no rotation/scale fit',
                  geometry_definition='sum of final-iteration accepted unit plane normals outer products; '
                    'a position-only local geometry diagnostic, not full pose observability', cases=results)
    (args.group/'sampling-summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    with (args.group/'sampling-summary.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(('label','E_rmse_m','N_rmse_m','U_rmse_m','3d_rmse_m',
                         'prevoxel_mean','voxel_mean','effective_mean','normal_E','normal_N','normal_U',
                         'mean_abs_residual_m','condition_median','solve_ms_median'))
        for r in results:
            writer.writerow((r['label'], *r['rmse_enu_m'], r['rmse_3d_m'], r['points_prevoxel_mean'],
                             r['points_voxel_mean'],r['effective_points_mean'],
                             *r['normal_axis_fraction_mean'],r['mean_abs_residual_m'],
                             r['normal_gram_condition_median'],r['solve_ms_median']))
    with (args.group/'sampling-error-timeseries.csv').open('w',newline='') as file:
        writer=csv.writer(file)
        writer.writerow(('case','sim_time_s','east_error_m','north_error_m','up_error_m','attitude_error_deg'))
        for case in cases:
            for row in case['error_rows']:
                writer.writerow((case['config']['label'],*row))
    plt.rcParams.update({'font.family':'Noto Sans CJK SC','font.size':10})
    plot_cases=[case for case in cases if not case['config']['label'].endswith('-neighborhood')]
    plot_results=[r for r in results if not r['label'].endswith('-neighborhood')]
    fig, axes = plt.subplots(3,1,figsize=(13,10),sharex=True)
    for case in plot_cases:
        rows=case['error_rows']
        for axis,ax in enumerate(axes):
            ax.plot(rows[:,0]-start,rows[:,axis+1],label=case['config']['label'],linewidth=1.4)
    for ax,name in zip(axes,('东 E','北 N','天 U')):
        ax.set_ylabel(name+'误差 / m');ax.grid(alpha=.25)
    axes[0].legend(ncol=3,fontsize=8)
    axes[-1].set_xlabel('公共区间内仿真时间 / s')
    fig.suptitle('同录包抽稀消融：纯 LiDAR/IMU ENU 误差',fontsize=16)
    fig.tight_layout(rect=(0,0,1,.96))
    fig.savefig(args.group/'sampling-enu-errors.png',dpi=170)
    fig.savefig(args.group/'sampling-enu-errors.pdf')
    plt.close(fig)
    fig, axes = plt.subplots(2,2,figsize=(14,9))
    names=[r['label'] for r in plot_results];x=np.arange(len(plot_results))
    axes[0,0].bar(x,[r['rmse_3d_m'] for r in plot_results],color='#168aad')
    axes[0,0].set_ylabel('三维 RMSE / m')
    for key,label in [('points_prevoxel_mean','体素前'),('points_voxel_mean','体素后'),('effective_points_mean','有效匹配')]:
        axes[0,1].plot(x,[r[key] for r in plot_results],marker='o',label=label)
    axes[0,1].set_ylabel('每帧平均点数');axes[0,1].legend()
    bottom=np.zeros(len(plot_results))
    for axis,label in enumerate(('东西法向','南北法向','垂直法向')):
        vals=[r['normal_axis_fraction_mean'][axis] for r in plot_results]
        axes[1,0].bar(x,vals,bottom=bottom,label=label);bottom+=vals
    axes[1,0].set_ylabel('有效匹配平面法向主轴占比');axes[1,0].legend(fontsize=8)
    axes[1,1].bar(x,[r['normal_gram_condition_median'] for r in plot_results],color='#e67700')
    axes[1,1].set_yscale('log');axes[1,1].set_ylabel('位置法向矩阵条件数（中位数）')
    for ax in axes.flat:
        ax.set_xticks(x,names,rotation=35,ha='right');ax.grid(axis='y',alpha=.2)
    fig.suptitle('抽稀后的误差、点数与匹配几何',fontsize=16)
    fig.tight_layout(rect=(0,0,1,.96))
    fig.savefig(args.group/'sampling-diagnostic-summary.png',dpi=170)
    fig.savefig(args.group/'sampling-diagnostic-summary.pdf')
    plt.close(fig)
    quality_cases=[case for case in cases if 'line_neighborhood_fraction' in case['diag'].dtype.names]
    if quality_cases:
        fig,axes=plt.subplots(3,1,figsize=(13,10),sharex=True)
        for case in quality_cases:
            diag=case['diag'];mask=(diag['stamp_s']>=start)&(diag['stamp_s']<=end);diag=diag[mask]
            for ax,field in zip(axes,('line_neighborhood_fraction','line_neighborhood_U_fraction',
                                      'mean_abs_residual')):
                ax.plot(diag['stamp_s']-start,diag[field],label=case['config']['label'],linewidth=1.3)
        for ax,label in zip(axes,('有效匹配中近共线邻域占比','垂直法向匹配中近共线占比','平均绝对点面残差 / m')):
            ax.set_ylabel(label);ax.grid(alpha=.25)
        axes[0].legend(ncol=3,fontsize=9)
        axes[-1].set_xlabel('公共区间内仿真时间 / s')
        fig.suptitle(r'五近邻平面质量诊断：近共线判据 $\lambda_2/\lambda_3 < 0.01$',fontsize=16)
        fig.tight_layout(rect=(0,0,1,.96))
        fig.savefig(args.group/'plane-neighborhood-quality.png',dpi=170)
        fig.savefig(args.group/'plane-neighborhood-quality.pdf')
        plt.close(fig)
    indexed={r['label']:r for r in results}
    comparisons=[
        ('采样方式：同一录包、相同算法参数',[
            ('full','全部点'),('stride2-even','半采样：偶数点'),('stride2-odd','半采样：奇数点'),
            ('hash-half','随机半采样'),('stride3-retry','每三点取一'),('stride4','每四点取一')]),
        ('独立干预：检查哪些机制能复现改善',[
            ('full','全部点'),('full-cov2','全部点：噪声方差×2'),
            ('half-covhalf','半采样：噪声方差÷2'),('full-reject-line005','全部点：筛除窄邻域'),
            ('full-map-voxel1','全部点：地图体素1m'),('full-scan-voxel1','全部点：扫描体素1m')])]
    fig,axes=plt.subplots(2,1,figsize=(12,9))
    for ax,(title,items) in zip(axes,comparisons):
        items=[item for item in items if item[0] in indexed]
        bars=ax.bar(np.arange(len(items)),[indexed[key]['rmse_3d_m'] for key,_ in items],
                    color=['#d6336c' if key=='full' else '#168aad' for key,_ in items])
        ax.bar_label(bars,fmt='%.3f m',padding=4,fontsize=11)
        ax.set_xticks(np.arange(len(items)),[name for _,name in items],rotation=15,ha='right')
        ax.set_ylabel('三维 RMSE / m');ax.set_title(title,fontsize=14)
        ax.set_ylim(0,max(indexed[key]['rmse_3d_m'] for key,_ in items)*1.25)
        ax.grid(axis='y',alpha=.2)
    fig.suptitle('抽稀收益的受控诊断结果',fontsize=18)
    fig.text(.5,.01,'共同区间 5.58–34.38 s；仅作首点平移对齐；真值不参与 LIO 解算。',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.03,1,.96))
    fig.savefig(args.group/'sampling-focused-results.png',dpi=180)
    fig.savefig(args.group/'sampling-focused-results.pdf')
    plt.close(fig)
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
