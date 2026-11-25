#!/usr/bin/env python3
"""Independent, offline SITL return verification. Truth is evaluation-only."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'build/px4-pydeps'))


def verify(run):
    import rosbag
    from pyulog import ULog
    nav=json.loads((run/'navigation-report.json').read_text())
    truth=[];confirmed=0
    with rosbag.Bag(str(run/'sensors.bag')) as bag:
        for topic,msg,_ in bag.read_messages(topics=['/ground_truth/state','/msim/navigation/status']):
            if topic=='/ground_truth/state':
                p=msg.pose.pose.position;truth.append([msg.header.stamp.to_sec(),p.x,p.y,p.z])
            else:confirmed+=bool(json.loads(msg.data).get('localization',{}).get('accepted'))
    data=np.asarray(truth);errors=[]
    if len(data)<100:raise ValueError('Insufficient independent truth samples')
    home=data[0,1:];final=data[data[:,0]>data[-1,0]-.5,1:].mean(0)
    final_error=float(np.linalg.norm((final-home)[:2]));climb=float(np.max(data[:,3]-home[2]))
    excursion=float(np.max(np.linalg.norm(data[:,1:3]-home[:2],axis=1)))
    flags=[];armed_states=[];land_states=[]
    logs=sorted((run/'ros-home/log').rglob('*.ulg'))
    for path in logs:
        u=ULog(str(path),message_name_filter_list=['estimator_status_flags','vehicle_status','vehicle_land_detected'])
        for dataset in u.data_list:
            d=dataset.data
            if dataset.name=='estimator_status_flags' and dataset.multi_id==0:
                airborne=d['cs_in_air'].astype(bool)
                flags.extend([{k:bool(d[k][i]) for k in ['cs_ev_pos','cs_baro_hgt','cs_gnss_pos','cs_gps_hgt','cs_gnss_vel']} for i in np.flatnonzero(airborne)])
            elif dataset.name=='vehicle_status':armed_states.extend(map(int,d['arming_state']))
            elif dataset.name=='vehicle_land_detected':land_states.extend(map(bool,d['landed']))
    ev_fraction=sum(f['cs_ev_pos'] for f in flags)/max(1,len(flags))
    baro_fraction=sum(f['cs_baro_hgt'] for f in flags)/max(1,len(flags))
    gnss_active=any(f['cs_gnss_pos'] or f['cs_gps_hgt'] or f['cs_gnss_vel'] for f in flags)
    rows=[json.loads(line) for line in (run/'injected-gps.jsonl').read_text().splitlines() if line.strip()]
    max_attack=max((abs(r.get('attack_east_m',0)) for r in rows),default=0.)
    landed=bool(armed_states and armed_states[-1]!=2 and land_states and land_states[-1])
    if not nav['completed'] or nav['failures']:errors.append('controller did not complete a successful return')
    if nav.get('truth_used_for_control') is not False:errors.append('truth exclusion not declared')
    goal=nav.get('goal_relative_enu_m')
    landing_limit=3. if goal is not None else 1.5
    if final_error>landing_limit:errors.append(f'landing is more than {landing_limit} m from takeoff')
    goal_error=None;goal_minimum=None;goal_truth=None;goal_horizontal_error=None;goal_vertical_error=None
    goal_radius=float(nav.get("goal_radius_m",2.))
    horizontal_limit=goal_radius+3.
    if goal is not None:
        target=home+np.asarray(goal)
        distances=np.linalg.norm(data[:,1:]-target,axis=1);goal_minimum=float(np.min(distances))
        when=nav.get('goal_reached_sim_time_s')
        if when is not None:
            index=int(np.argmin(abs(data[:,0]-when)))
            goal_truth=data[index,1:].tolist();goal_error=float(distances[index])
            goal_horizontal_error=float(np.linalg.norm((data[index,1:]-target)[:2]));goal_vertical_error=float(abs(data[index,3]-target[2]))
        if not nav.get('goal_reached') or goal_horizontal_error is None or goal_horizontal_error>horizontal_limit or goal_vertical_error>3.:
            errors.append('goal region arrival is not confirmed by independent truth')
    if climb<3 or excursion<4:errors.append('insufficient airborne outbound motion')
    if not logs:errors.append('PX4 ULog unavailable; aiding/landing checks cannot be confirmed')
    if flags and (ev_fraction<.95 or baro_fraction<.95 or gnss_active):errors.append('flight aiding selectors did not match LIO horizontal + barometric height')
    if logs and not flags:errors.append('ULog has no airborne EKF samples')
    if logs and not landed:errors.append('ULog does not confirm landed and disarmed')
    if nav.get('prior_map') and (not nav.get('prior_map_confirmed') or confirmed<2):errors.append('prior-map localization not confirmed in bag')
    return dict(pass_all=not errors,run=str(run),evaluation_only_truth=True,truth_samples=len(data),
        landing_horizontal_error_m=final_error,final_displacement_enu_m=(final-home).tolist(),
        maximum_climb_m=climb,maximum_horizontal_excursion_m=excursion,
        goal_relative_enu_m=goal,goal_truth_enu_m=goal_truth,goal_error_at_arrival_m=goal_error,
        minimum_truth_distance_to_goal_m=goal_minimum,goal_radius_m=goal_radius,goal_horizontal_error_at_arrival_m=goal_horizontal_error,goal_vertical_error_at_arrival_m=goal_vertical_error,
        goal_horizontal_truth_tolerance_m=horizontal_limit,goal_vertical_truth_tolerance_m=3.,
        landing_tolerance_m=landing_limit,
        airborne_ekf_flag_samples=len(flags),ev_horizontal_active_fraction=ev_fraction,
        barometric_height_active_fraction=baro_fraction,gnss_active_in_air=gnss_active,
        landed_disarmed_in_ulog=landed,prior_map_confirmed=nav.get('prior_map_confirmed'),
        confirmed_map_status_messages=confirmed,attack_active_epochs=sum(bool(r.get('attack_active')) for r in rows),
        maximum_injected_east_offset_m=max_attack,errors=errors)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path,help='Report path; default is inside the recording directory')
    args=parser.parse_args();run=args.run.resolve();report=verify(run)
    output=args.output or run/'autonomy-verification.json';output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['pass_all']:raise SystemExit(1)


if __name__=='__main__':main()
