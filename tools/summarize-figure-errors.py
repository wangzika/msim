#!/usr/bin/env python3
"""Summarize the actual error samples behind the delivered figures."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
METHODS = {
    '/msim/spoofed_gnss_odom': 'GPS',
    '/Odometry': 'LiDAR/IMU',
    '/loose_odom': 'LiDAR/IMU/GPS 松组合',
    'fast-lio-velodyne': 'FAST-LIO',
    'lio-sam-patched': 'LIO-SAM',
    'ligo': 'LIGO',
    'glio-front-end': 'GLIO 前端参考',
}


def metrics(rows):
    rows = np.asarray(sorted(rows, key=lambda row: row[0]), dtype=float)
    if not len(rows) or not np.isfinite(rows).all():
        raise ValueError('missing or non-finite error samples')
    error = rows[:, 1:]
    distance = np.linalg.norm(error, axis=1)
    rmse = np.sqrt(np.mean(error**2, axis=0))
    result = dict(samples=len(rows), start_s=float(rows[0, 0]), end_s=float(rows[-1, 0]),
                  rmse_e_m=float(rmse[0]), rmse_n_m=float(rmse[1]), rmse_u_m=float(rmse[2]),
                  rmse_horizontal_m=float(np.sqrt(np.mean(np.sum(error[:, :2]**2, axis=1)))),
                  rmse_3d_m=float(np.sqrt(np.mean(distance**2))),
                  mean_3d_m=float(np.mean(distance)), std_3d_m=float(np.std(distance)),
                  p95_3d_m=float(np.percentile(distance, 95)), max_3d_m=float(np.max(distance)),
                  final_3d_m=float(distance[-1]),
                  max_error_time_s=float(rows[np.argmax(distance), 0]))
    for prefix, values in [('mean', error.mean(0)), ('std', error.std(0)),
                           ('max_abs', np.abs(error).max(0)), ('final', error[-1])]:
        result.update({prefix+'_'+axis+'_m': float(value) for axis, value in zip(('e', 'n', 'u'), values)})
    assert np.isclose(result['rmse_3d_m']**2, sum(result['rmse_'+axis+'_m']**2 for axis in ('e', 'n', 'u')))
    return result


def load_csv(path, id_column, axes):
    data = {}
    with path.open() as file:
        for row in csv.DictReader(file):
            data.setdefault(row[id_column], []).append(
                [float(row['time_since_gps_source_s']), *[float(row[key]) for key in axes]])
    return {name: np.asarray(rows) for name, rows in data.items()}


def number(value):
    return f'{0.0 if abs(value)<.0005 else value:.3f}'


def table(rows, fields):
    lines = ['| 方法 | '+' | '.join(label for key, label in fields)+' |',
             '| --- | '+' | '.join('---:' for _ in fields)+' |']
    for label, values in rows:
        lines.append('| '+label+' | '+' | '.join(
            str(values[key]) if key == 'samples' else number(values[key]) for key, _ in fields)+' |')
    return lines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, default=ROOT/'runs/figure-metrics-20261118')
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    families = {
        'GPS、LIO 与松组合图': ROOT/'runs/presentation-four-figures-20261118',
        '不同算法时序图': ROOT/'runs/lio-spoof-algorithms-20261118/figures',
    }
    all_data, records = {}, []
    for family, directory in families.items():
        algorithm = family == '不同算法时序图'
        for condition in ('clean', 'spoofed'):
            data = load_csv(directory/(condition+'-errors.csv'), 'algorithm' if algorithm else 'series',
                            ('east_error_m', 'north_error_m', 'up_error_m') if algorithm
                            else ('east_m', 'north_m', 'up_m'))
            for method, rows in data.items():
                all_data[(family, condition, method)] = rows
                records.append(dict(family=family, condition=condition, method=METHODS[method],
                                    source_csv=str(directory/(condition+'-errors.csv')),
                                    scope='图中实际有效样本', **metrics(rows)))
    # The first stand-alone figure used another flight and another point stride.
    spec = importlib.util.spec_from_file_location('comparison', ROOT/'tools/compare-fastglio-spoof-runs.py')
    comparison = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(comparison)
    old_meta = json.loads((ROOT/'runs/presentation-3d-20261118/gps-lio-loose-truth-3d.json').read_text())
    old = comparison.read_run(Path(old_meta['source_run']))
    start, end = old_meta['common_interval_s']
    for method, rows in old['errors'].items():
        rows = rows[(rows[:, 0]>=start)&(rows[:, 0]<=end)]
        records.append(dict(family='最初单独生成的三维图', condition='spoofed', method=METHODS[method],
                            source_run=old_meta['source_run'], scope='图中实际有效样本', **metrics(rows)))
    # Validate numbers against each plotting job's saved summary.
    fusion_summary = json.loads((families['GPS、LIO 与松组合图']/'figure-sources.json').read_text())
    alg_summary = json.loads((families['不同算法时序图']/'summary-symlog.json').read_text())
    for (family, condition, method), rows in all_data.items():
        expected = (alg_summary['results'][condition][method] if family == '不同算法时序图'
                    else fusion_summary['runs'][condition]['error_metrics'][method])
        assert np.isclose(metrics(rows)['rmse_3d_m'], expected['rmse_3d_m'])
    common_start = max(rows[0, 0] for rows in all_data.values())
    common_end = min(rows[-1, 0] for rows in all_data.values())
    common = []
    for (family, condition, method), rows in all_data.items():
        selected = rows[(rows[:, 0]>=common_start)&(rows[:, 0]<=common_end)]
        common.append(dict(family=family, condition=condition, method=METHODS[method],
                           scope='当前两类图的共同时间窗', **metrics(selected)))
    result = dict(unit='m', records=records, common_window_s=[common_start, common_end],
                  common_window_records=common,
                  note='sample-weighted metrics; no resampling, rotation, scale fitting or smoothing; '
                  'GPS errors use antenna truth, other outputs use body truth; nonlinear axes do not alter metrics')
    (out/'metrics.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    for filename, values in [('metrics.csv', records), ('metrics-common-window.csv', common)]:
        keys = list(dict.fromkeys(key for row in values for key in row))
        with (out/filename).open('w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=keys)
            writer.writeheader()
            writer.writerows(values)
    fields = [('rmse_e_m','东 E RMSE'),('rmse_n_m','北 N RMSE'),('rmse_u_m','天 U RMSE'),
              ('rmse_3d_m','三维 RMSE'),('max_3d_m','最大三维误差'),('final_3d_m','末端三维误差')]
    extra = [('samples','有效样本数'),('start_s','开始/s'),('end_s','结束/s'),
             ('mean_3d_m','平均三维误差'),('p95_3d_m','95%分位误差'),('std_3d_m','三维误差标准差')]
    doc = ['# 已生成图表的误差指标（2026-11-18）','',
           '全部位置误差单位为米。各估计轨迹按首个真值匹配点平移对齐，不做旋转、尺度拟合、平滑或重采样；真值配对时间差上限为 50 ms。下方主表按每张图实际绘制的有效样本统计，因此每条曲线的起止时刻可能不同。', '',
           '三维 RMSE = sqrt(mean(E²+N²+U²))，也等于 sqrt(RMSE_E²+RMSE_N²+RMSE_U²)。最大误差、末端误差、平均误差、95%分位和标准差均对三维误差模长计算，标准差取总体标准差。末端指该曲线最后一个有效样本，不一定是所有算法相同的时刻。真实轨迹是参考，其自身误差按定义为 0。GPS 使用天线真值，其余输出使用机体真值。', '',
           '三维图和对应 ENU 图共享一组轨迹；紧凑版只调整留白，指标不变。不同算法图的线性版和非线性版也共享相同数据，指标不重复计算。', '']
    for family in [*families, '最初单独生成的三维图']:
        doc += ['## '+family,'']
        if family == 'GPS、LIO 与松组合图':
            doc += ['对应 `presentation-four-figures-20261118` 中的三维图、紧凑三维图及 ENU 时序图。使用 simple 场景下的成对在线飞行，FAST-LIO 抽稀参数为 4。','']
        elif family == '不同算法时序图':
            doc += ['对应 `lio-spoof-algorithms-20261118/figures` 中的线性和对称对数纵轴图。四种前端在相同源录包上离线半速回放，不输入 GPS。该 FAST-LIO 曲线是重新回放结果，不能与在线曲线混为同一条。LIO-SAM 使用 IMU 姿态；GLIO 仅为前端参考。','']
        else:
            doc += ['对应 `presentation-3d-20261118` 最初单独生成的三维图，包括后续添加真实轨迹的版本。源数据为 `uav-px4-20261116-181930`，抽稀参数为 2，与上述成对实验不是同一次运行。','']
        for condition, label in [('spoofed','有欺骗飞行数据'),('clean','无欺骗飞行数据')]:
            chosen = [r for r in records if r['family']==family and r['condition']==condition]
            if not chosen:
                continue
            order = ['GPS','LiDAR/IMU','LiDAR/IMU/GPS 松组合'] if family != '不同算法时序图' else ['FAST-LIO','LIO-SAM','LIGO','GLIO 前端参考']
            chosen.sort(key=lambda r:order.index(r['method']))
            pairs = [(r['method'],r) for r in chosen]
            doc += ['### '+label,'',*table(pairs,fields),'',*table(pairs,extra),'']
    doc += ['## 当前图表的共同时间窗对照','',
            '另取 %.6f–%.6f s 的共同覆盖时间窗，供排除统计时长差异后的参考。仍按各算法有效输出样本计算，未强制重采样到同一频率；只裁剪时段，不重新归零。'% (common_start,common_end),'']
    for family in families:
        for condition,label in [('spoofed','有欺骗'),('clean','无欺骗')]:
            chosen=[(r['method'],r) for r in common if r['family']==family and r['condition']==condition]
            doc += ['### '+family+'／'+label,'',*table(chosen,fields),'']
    doc += ['## 文件','', '[完整数值 CSV](metrics.csv) · [共同时间窗 CSV](metrics-common-window.csv) · [完整 JSON](metrics.json)', '',
            '无欺骗 GPS 由仿真真值生成且未额外叠加随机定位噪声，因此误差接近浮点数值精度，不能解释为实物 GPS 达到零误差。有、无欺骗为两次独立飞行，算法初始化和实际运动也不同，不能把差异全部归因于欺骗。','']
    (out/'指标汇总.md').write_text('\n'.join(doc))
    print(out/'指标汇总.md')
    for family in families:
        for condition in ('spoofed','clean'):
            print(family,condition)
            for r in records:
                if r['family']==family and r['condition']==condition:
                    print(r['method'],*[number(r[k]) for k,_ in fields])


if __name__ == '__main__':
    main()
