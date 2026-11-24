#!/usr/bin/env python3
"""Check a completed model-matched GPS capture without changing its measurements."""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'build/px4-pydeps'))


def verify(run):
    import rosbag
    from pyulog import ULog
    profile=json.loads((run/'sensor_profile.json').read_text());g=profile['gps']
    if g.get('reported_variance_mode')!='model_marginal':
        raise ValueError('This check requires a model_marginal GPS preset')
    expected=[a*a+b*b for a,b in zip(g['position_sigma_enu_m'],g['bias_sigma_enu_m'])]
    rows=[json.loads(line) for line in (run/'injected-gps.jsonl').read_text().splitlines() if line.strip()]
    errors=[];by_stamp={r['sim_time_ns']:r for r in rows}
    for row in rows:
        if row.get('environment_model')!='disabled' or row.get('environment_noise_scale')!=1:
            errors.append('unexpected environment scaling');break
        if row.get('position_error_model_id')!=profile['gps_reference']['id']:
            errors.append('unexpected receiver model ID');break
        if any(not math.isclose(a,b,rel_tol=1e-9) for a,b in zip(row['reported_variance_enu_m2'],expected)):
            errors.append('logged covariance differs from model');break
    samples={'/msim/normal_gnss_odom':{},'/msim/spoofed_gnss_odom':{}}
    with rosbag.Bag(str(run/'sensors.bag')) as bag:
        for topic,msg,_ in bag.read_messages(topics=list(samples)):
            p=msg.pose.pose.position;stamp=msg.header.stamp.to_nsec()
            samples[topic][stamp]=[p.x,p.y,p.z]
            if any(not math.isclose(msg.pose.covariance[k],v,rel_tol=1e-9) for k,v in zip([0,7,14],expected)):
                errors.append('ROS covariance differs from model');break
    normal,spoofed=samples.values();pairs=0
    for stamp in normal.keys() & spoofed.keys() & by_stamp.keys():
        shift=[by_stamp[stamp][k] for k in ['attack_east_m','attack_north_m','attack_up_m']]
        if any(abs((b-a)-s)>1e-6 for a,b,s in zip(normal[stamp],spoofed[stamp],shift)):
            errors.append('normal/spoofed branches do not share one realization');break
        pairs+=1
    gps_count=0;expected_accuracy=[math.sqrt(max(expected[:2])),math.sqrt(expected[2])]
    for path in (run/'ros-home/log').rglob('*.ulg'):
        ulog=ULog(str(path),message_name_filter_list=['sensor_gps'])
        for dataset in ulog.data_list:
            if dataset.name!='sensor_gps' or dataset.multi_id!=0:continue
            data=dataset.data
            for i in range(len(data['timestamp'])):
                if data['fix_type'][i]<3:continue
                gps_count+=1
                if abs(data['eph'][i]-expected_accuracy[0])>.011 or abs(data['epv'][i]-expected_accuracy[1])>.011:
                    errors.append('PX4 reported accuracy differs from model (including MAVLink quantization)');break
    if g['native_noise'] or g['adaptive_environment']:errors.append('double noise or uncalibrated scaling enabled')
    if not rows or not pairs or not gps_count:errors.append('missing GPS logs, paired ROS measurements or PX4 GPS samples')
    return dict(pass_all=not errors,run=str(run),receiver_model=profile['gps_reference']['id'],
                expected_covariance_enu_m2=expected,expected_eph_epv_m=expected_accuracy,
                logged_epochs=len(rows),paired_ros_epochs=pairs,px4_gps_samples=gps_count,
                native_noise=g['native_noise'],adaptive_environment=g['adaptive_environment'],errors=errors)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();run=args.run.resolve();report=verify(run)
    (run/'gnss-receiver-verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['pass_all']:raise SystemExit(1)


if __name__=='__main__':main()
