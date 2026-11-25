#!/usr/bin/env python3
"""Opt-in SITL LIO navigation, prior-map localization and recorded-route return.

No simulator truth subscription. A single process owns MAVLink control and EV.
"""
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

os.environ['MAVLINK20']='1'
import numpy as np
from scipy.spatial.transform import Rotation
from autonomy_core import FLU_FRD,MapLocalizer,bounded_target,segment_clear,transform,param_int_wire,param_int_value,fresh_stamp
from lio_map_recorder import cloud_points
from teleop_controls import CommandBuffer
from exploration_planner import ExplorationPlanner,advance_return_route


def main():
    import rospy
    from nav_msgs.msg import Odometry,Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from pymavlink import mavutil
    rospy.init_node('msim_px4_autonomy',disable_signals=True)
    run=Path(os.environ['MSIM_TELEOP_RUN']);stop_file=Path(os.environ['MSIM_TELEOP_STOP_FILE'])
    height=float(os.environ.get('MSIM_TELEOP_HEIGHT_M','8'))
    distance=float(os.environ.get('MSIM_AUTONOMY_DISTANCE_M','8'))
    return_after=float(os.environ.get('MSIM_RETURN_AFTER_S','30'))
    maximum=float(os.environ.get('MSIM_AUTONOMY_TIMEOUT_S','150'))
    goal_text=os.environ.get('MSIM_GOAL_ENU','')
    goal_offset=np.array([float(v) for v in goal_text.split(',')]) if goal_text else None
    mission_speed=float(os.environ.get('MSIM_MISSION_SPEED_MPS','2'))
    goal_radius=float(os.environ.get('MSIM_GOAL_RADIUS_M','2'))
    bundle=os.environ.get('MSIM_LOCALIZATION_MAP','')
    localizer=MapLocalizer(bundle) if bundle else None
    if localizer and localizer.info.get('scene')!=os.environ.get('MSIM_SCENE_KEY'):
        raise ValueError('Prior map belongs to a different scene')
    lock=threading.Lock();commands=CommandBuffer()
    shared=dict(odom=None,cloud=None,localization=dict(accepted=False,reason='mapping_without_prior'),localization_at=-math.inf,gps_alarm=False,
                planner_height=None,planning_goal=None,planning_radius=0.,grid=None,plan=dict(path=[],reason='waiting_for_takeoff'),plan_at=-math.inf)
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def odometry(msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        xyz=np.array([p.x,p.y,p.z]);quat=np.array([q.x,q.y,q.z,q.w])
        if msg.header.frame_id!='camera_init' or not np.isfinite(xyz).all() or not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1)>.05:return
        with lock:shared['odom']=(msg.header.stamp.to_sec(),xyz,quat)
    def cloud(msg):
        if msg.header.frame_id!='camera_init':return
        p=cloud_points(msg)[:,:3]
        with lock:shared['cloud']=(msg.header.stamp.to_sec(),p[np.isfinite(p).all(1)])
    def command(msg):
        try:commands.put(json.loads(msg.data))
        except (ValueError,TypeError):rospy.logwarn('Invalid autonomy command')
    def gps(msg):
        try:
            row=json.loads(msg.data)
            with lock:shared['gps_alarm']=row.get('detector_status')=='alarm'
        except ValueError:pass
    subscribers=[rospy.Subscriber('/Odometry',Odometry,odometry,queue_size=5),
                 rospy.Subscriber('/cloud_registered',PointCloud2,cloud,queue_size=1,buff_size=8*1024*1024),
                 rospy.Subscriber('/msim/teleop/command',String,command,queue_size=5),
                 rospy.Subscriber('/gnss/spoof_detection',String,gps,queue_size=5)]
    nav_pub=rospy.Publisher('/msim/navigation/status',String,queue_size=2,latch=True)
    teleop_pub=rospy.Publisher('/msim/teleop/status',String,queue_size=2,latch=True)
    route_pub=rospy.Publisher('/msim/return_path',RosPath,queue_size=1,latch=True)
    plan_pub=rospy.Publisher('/msim/planned_path',RosPath,queue_size=1,latch=True)
    localized_pub=rospy.Publisher('/msim/localized_odom',Odometry,queue_size=2)
    def locate_loop():
        previous=-math.inf
        while running and not rospy.is_shutdown():
            with lock:o=shared['odom'];c=shared['cloud']
            if o and c and c[0]-previous>1 and abs(o[0]-c[0])<.16:
                previous=c[0]
                try:result=localizer.locate(c[1],o[1])
                except (ValueError,np.linalg.LinAlgError) as error:result=dict(accepted=False,reason=str(error))
                with lock:
                    shared['localization']=result
                    if result['accepted']:shared['localization_at']=c[0]
            time.sleep(.05)
    worker=threading.Thread(target=locate_loop,daemon=True) if localizer else None
    if worker:worker.start()
    def planning_loop():
        planner=None;previous=-math.inf
        while running and not rospy.is_shutdown():
            with lock:
                o=shared['odom'];c=shared['cloud'];z=shared['planner_height'];goal=shared['planning_goal'];region=shared['planning_radius']
            if z is not None and planner is None:planner=ExplorationPlanner(z)
            # Build the horizontal cruise map only after takeoff. Otherwise a
            # moving slice during ascent permanently inserts the launch floor.
            if planner is not None and goal is not None and o and c and c[0]-previous>.45 and abs(o[0]-c[0])<.16:
                previous=c[0]
                try:
                    planner.update(o[1],c[1])
                    if goal is not None:
                        result=planner.plan(o[1],goal,goal_radius=region)
                        result['goal']=list(goal)
                        with lock:shared['plan']=result;shared['plan_at']=c[0];shared['grid']=planner.snapshot()
                        message=RosPath();message.header.frame_id='camera_init';message.header.stamp=rospy.Time.from_sec(c[0])
                        for p in result['path']:
                            item=PoseStamped();item.header=message.header;item.pose.position.x,item.pose.position.y,item.pose.position.z=p;item.pose.orientation.w=1.;message.poses.append(item)
                        plan_pub.publish(message)
                except (ValueError,MemoryError) as error:
                    with lock:shared['plan']=dict(path=[],reason=str(error));shared['plan_at']=c[0]
            time.sleep(.03)
    planning_worker=threading.Thread(target=planning_loop,daemon=True) if goal_offset is not None else None
    if planning_worker:planning_worker.start()
    master=mavutil.mavlink_connection('udpin:0.0.0.0:14540',source_system=255)
    state='waiting_lio';reason='startup';armed=False;mode='等待 PX4';local=None;attitude=None;measured_velocity=None;px4_messages=[];heartbeat_at=-math.inf;position_at=-math.inf
    origin=None;anchor=None;rotation=None;boot=None;armed_at=None;next_action=0.;last_report=0.;last_ev=-math.inf
    configured=False;breadcrumbs=[];return_route=[];visited=[];hold=None;last_transform=None
    started_wall=time.monotonic()
    ev_sent=0;events=[];failures=[];land_commanded=False;alarm_since=None;pause_since=None;previous_state=None
    goal_reached=False;goal_at=None;mission_goal=None;last_velocity=np.zeros(2);last_velocity_time=None
    unfresh_since=None
    position_mask=(mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE|mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE|
                   mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE|mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE|
                   mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE|mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE|
                   mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE|mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    log=(run/'navigation-events.jsonl').open('w',buffering=1)
    def transition(new,why,now):
        nonlocal state,reason,next_action
        if state==new:return
        state=new;reason=why;next_action=0.
        row=dict(state=new,reason=why,sim_time_s=now);events.append(row);log.write(json.dumps(row)+'\n');print(row,flush=True)
    def set_parameter(name,value):
        for _ in range(4):
            master.mav.param_set_send(master.target_system,1,name.encode(),param_int_wire(value),mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            until=time.monotonic()+1
            while time.monotonic()<until:
                msg=master.recv_match(type='PARAM_VALUE',blocking=True,timeout=.1)
                if msg:
                    key=msg.param_id.decode() if isinstance(msg.param_id,bytes) else msg.param_id
                    if key.rstrip('\0')==name and msg.param_type==mavutil.mavlink.MAV_PARAM_TYPE_INT32 and param_int_value(msg.param_value)==value:return
        raise RuntimeError('PX4 parameter not acknowledged: '+name)
    def begin_return(why,now,position):
        nonlocal return_route
        route=[p.copy() for p in reversed(breadcrumbs)]
        home=anchor.copy();home[2]+=height
        if localizer and last_transform is not None:
            home=transform(np.array([localizer.info['home_position']]),np.linalg.inv(last_transform))[0];home[2]+=height
        route.append(home);return_route=route
        message=RosPath();message.header.frame_id='camera_init';message.header.stamp=rospy.Time.now()
        for p in route:
            item=PoseStamped();item.header=message.header;item.pose.position.x,item.pose.position.y,item.pose.position.z=p;item.pose.orientation.w=1.;message.poses.append(item)
        route_pub.publish(message);transition('returning',why,now)
    def finalize(now):
        nonlocal running,state,reason
        if goal_offset is not None and not goal_reached and not failures:failures.append('goal_not_reached')
        if failures:reason=failures[-1]
        report=dict(completed=land_commanded and reason=='home_reached' and not failures and not armed,landed_disarmed=not armed,reason=reason,events=events,
                    failures=failures,external_pose_messages=ev_sent,gnss_fusion_enabled=False,
                    prior_map=bundle or None,prior_map_confirmed=last_transform is not None,
                    home_lio=anchor.tolist() if anchor is not None else None,truth_used_for_control=False,
                    mission_type='goal_exploration' if goal_offset is not None else 'recorded_return',
                    goal_relative_enu_m=goal_offset.tolist() if goal_offset is not None else None,
                    goal_reached=goal_reached,goal_reached_sim_time_s=goal_at,goal_radius_m=goal_radius)
        if goal_offset is not None:report['completed']=report['completed'] and goal_reached
        (run/'navigation-report.json').write_text(json.dumps(report,indent=2)+'\n')
        state='landed' if report['completed'] else 'failed'
        for filename in ['navigation-status.json','teleop-status.json']:
            path=run/filename
            status=json.loads(path.read_text()) if path.exists() else {}
            status.update(state=state,reason=reason,armed=armed,sim_time_ns=int(now*1e9),error=failures[-1] if failures else None)
            tmp=run/(filename+'.tmp');tmp.write_text(json.dumps(status)+'\n');tmp.replace(path)
        stop_file.write_text(json.dumps(dict(reason='autonomy_complete' if report['completed'] else 'autonomy_failed',armed=False,sim_time_ns=int(now*1e9)))+'\n');running=False
    try:
        while running and not rospy.is_shutdown():
            now=rospy.Time.now().to_sec()
            if rotation is None and time.monotonic()-started_wall>70:raise RuntimeError('External navigation startup: no fresh LIO, attitude or local position')
            for _ in range(200):
                msg=master.recv_match(blocking=False)
                if msg is None:break
                if msg.get_type()=='HEARTBEAT' and msg.get_srcComponent()==1:
                    heartbeat_at=time.monotonic()
                    armed=bool(msg.base_mode&mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED);mode=mavutil.mode_string_v10(msg)
                elif msg.get_type()=='LOCAL_POSITION_NED':
                    position_at=time.monotonic();local=np.array([msg.x,msg.y,msg.z]);measured_velocity=np.array([msg.vx,msg.vy,msg.vz])
                elif msg.get_type()=='STATUSTEXT':
                    text=msg.text.decode(errors='replace') if isinstance(msg.text,bytes) else msg.text
                    row=dict(source='PX4',severity=int(msg.severity),text=text.rstrip('\0'),sim_time_s=now)
                    px4_messages.append(row);px4_messages=px4_messages[-30:]
                    with (run/'px4-events.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
                elif msg.get_type()=='ATTITUDE':attitude=[msg.roll,msg.pitch,msg.yaw]
                elif msg.get_type()=='TIMESYNC' and msg.tc1==0:
                    master.mav.timesync_send(rospy.Time.now().to_nsec(),msg.ts1)
            with lock:
                o=shared['odom'];c=shared['cloud'];loc=dict(shared['localization']);loc_at=shared['localization_at'];gps_alarm=shared['gps_alarm']
                plan=dict(shared['plan']);plan_at=shared['plan_at'];grid=shared['grid']
            # Read time after the callback snapshot, not before draining MAVLink.
            now=rospy.Time.now().to_sec()
            fresh=o is not None and fresh_stamp(now,o[0])
            unfresh_since=None if fresh else unfresh_since or time.monotonic()
            if rotation is None and fresh and attitude is not None and local is not None:
                anchor=o[1].copy();origin=local.copy();rotation=Rotation.from_euler('xyz',attitude).as_matrix()@FLU_FRD@Rotation.from_quat(o[2]).as_matrix().T
                hold=anchor+np.array([0,0,height]);boot=now
                if goal_offset is not None:
                    mission_goal=anchor+goal_offset
                    distance=float(np.linalg.norm(goal_offset[:2]))
                    with lock:shared['planner_height']=float(mission_goal[2])
                # These are dynamic EKF2 aiding selectors. No failsafe limits are disabled.
                for key,value in [('EKF2_GPS_CTRL',0),('EKF2_EV_CTRL',1),('EKF2_HGT_REF',0),('COM_ARM_WO_GPS',1)]:set_parameter(key,value)
                configured=True;transition('warming_up','external_position_initialization',now)
                (run/'external-navigation-config.json').write_text(json.dumps(dict(EKF2_GPS_CTRL=0,EKF2_EV_CTRL=1,EKF2_HGT_REF=0,
                    source='/Odometry',rotation_lio_to_ned=rotation.tolist(),origin_ned=origin.tolist(),anchor_lio=anchor.tolist(),position_std_floor_m=.1,truth_used=False),indent=2)+'\n')
            if fresh and rotation is not None and o[0]>last_ev:
                p=rotation@(o[1]-anchor)+origin
                angles=Rotation.from_matrix(rotation@Rotation.from_quat(o[2]).as_matrix()@FLU_FRD).as_euler('xyz')
                covariance=[0.]*21
                for index in [0,6,11]:covariance[index]=.01
                for index in [15,18,20]:covariance[index]=.01
                master.mav.vision_position_estimate_send(int(o[0]*1e6),*p,*angles,covariance,0)
                last_ev=o[0];ev_sent+=1
            if loc.get('accepted'):
                last_transform=np.asarray(loc['transform'])
                if o:
                    lp=transform(np.array([o[1]]),last_transform)[0]
                    message=Odometry();message.header.frame_id='prior_map';message.header.stamp=rospy.Time.from_sec(o[0]);message.pose.pose.position.x,message.pose.pose.position.y,message.pose.pose.position.z=lp
                    q=Rotation.from_matrix(last_transform[:3,:3]@Rotation.from_quat(o[2]).as_matrix()).as_quat()
                    message.pose.pose.orientation.x,message.pose.pose.orientation.y,message.pose.pose.orientation.z,message.pose.pose.orientation.w=q
                    localized_pub.publish(message)
            map_ok=not localizer or now-loc_at<5.
            for cmd,_ in commands.drain():
                op=cmd.get('op')
                if op=='return_home' and armed and o:begin_return('operator_return',now,o[1])
                elif op in ('land','finish'):
                    if armed:transition('landing','operator_land',now);land_commanded=True
                    else:failures.append('stopped_before_flight');finalize(now)
                elif op=='hold' and armed and o:
                    hold=o[1].copy();previous_state=state;pause_since=now;transition('holding','operator_hold',now)
            if configured and now-boot>maximum and state!='landing':
                failures.append('mission_timeout');transition('landing','mission_timeout',now);land_commanded=True
            if armed and not fresh and time.monotonic()-unfresh_since>.5 and state!='landing':
                failures.append('lio_timeout');transition('landing','lio_timeout',now);land_commanded=True
            if armed and fresh and state!='landing' and (np.linalg.norm(o[1][:2]-anchor[:2])>distance+15 or abs(o[1][2]-anchor[2])>height+5):
                failures.append('flight_envelope');transition('landing','flight_envelope',now);land_commanded=True
            if armed_at is not None and now-armed_at>2 and not armed and state not in ('landing',):
                failures.append('unexpected_disarm');finalize(now)
            if state=='warming_up':
                if now-boot>4 and fresh and map_ok:
                    # The EKF local vertical origin can change while switching height
                    # references on the ground. Anchor control to the settled baro z.
                    origin[2]=local[2]
                    (run/'control-origin.json').write_text(json.dumps(dict(origin_ned=origin.tolist(),home_lio=anchor.tolist(),height_source='barometer',sim_time_s=now))+'\n')
                    transition('arming','ready',now)
                elif now-boot>35 and not map_ok:failures.append('prior_map_not_confirmed');finalize(now)
            if state=='arming':
                if armed and mode=='OFFBOARD':armed_at=now;transition('taking_off','armed_with_external_position',now)
                elif now-boot>65:failures.append('arming_timeout');finalize(now)
                elif time.monotonic()>=next_action:
                    master.set_mode('OFFBOARD');master.arducopter_arm();next_action=time.monotonic()+2.
            baro_height=float(origin[2]-local[2]) if origin is not None and local is not None else None
            at_height=baro_height is not None and abs(baro_height-height)<1.5
            if state=='taking_off' and fresh and np.linalg.norm(o[1][:2]-hold[:2])<.45 and baro_height is not None and abs(baro_height-height)<.45:
                breadcrumbs=[o[1].copy()];transition('outbound','takeoff_reached',now)
            if state=='outbound' and fresh:
                hold=mission_goal.copy() if mission_goal is not None else anchor+np.array([0,distance,height])
                if not breadcrumbs or np.linalg.norm(o[1]-breadcrumbs[-1])>(1.5 if mission_goal is not None else .65):breadcrumbs.append(o[1].copy())
                alarm_since=(alarm_since or now) if gps_alarm else None
                if mission_goal is not None and np.linalg.norm(o[1][:2]-mission_goal[:2])<goal_radius and at_height and grid is not None and grid.known_clear(o[1],o[1]):
                    goal_reached=True;goal_at=now;begin_return('goal_reached',now,o[1])
                elif mission_goal is not None and now-boot>maximum*.6:
                    failures.append('outbound_time_budget');begin_return('outbound_time_budget',now,o[1])
                elif mission_goal is None and now-armed_at>return_after:begin_return('scheduled_return',now,o[1])
                elif alarm_since and now-alarm_since>2:begin_return('gnss_alarm_return',now,o[1])
            if state in ('outbound','returning') and not map_ok and fresh:
                previous_state=state;hold=o[1].copy();pause_since=now;transition('holding','localization_unconfirmed',now)
            if state=='holding' and reason=='localization_unconfirmed' and map_ok:transition(previous_state,'localization_recovered',now)
            if mission_goal is not None and state=='holding' and reason in ('planning_wait','obstacle_or_unknown_cloud'):
                expected=mission_goal if previous_state=='outbound' else return_route[0] if return_route else hold
                if plan.get('path') and now-plan_at<2. and np.linalg.norm(np.asarray(plan.get('goal',expected))-expected)<.1:
                    hold=expected.copy()
                    transition(previous_state,'replanned',now)
                elif now-pause_since>40 and previous_state=='outbound':
                    failures.append('exploration_stalled');begin_return('exploration_stalled',now,o[1])
            if state=='holding' and now-pause_since>(45 if mission_goal is not None else 15):
                failures.append(reason);transition('landing','hold_timeout',now);land_commanded=True
            if state=='returning' and fresh:
                if mission_goal is not None:
                    return_route,reached=advance_return_route(o[1],return_route,at_height,grid)
                    visited.extend(p.tolist() for p in reached)
                else:
                    while return_route and np.linalg.norm(o[1][:2]-return_route[0][:2])<.65 and at_height:
                        visited.append(return_route.pop(0).tolist())
                if return_route:hold=return_route[0]
                else:transition('landing','home_reached',now);land_commanded=True
            if mission_goal is not None and state in ('outbound','returning','holding'):
                planning_goal=mission_goal if state=='outbound' or state=='holding' and previous_state=='outbound' else return_route[0] if return_route else hold
                with lock:
                    shared['planning_goal']=planning_goal.tolist()
                    shared['planning_radius']=goal_radius if state=='outbound' or state=='holding' and previous_state=='outbound' else 1.5 if len(return_route)>1 else .3
            if state=='landing':
                with lock:shared['planning_goal']=None
                if not armed and land_commanded:finalize(now)
                elif time.monotonic()>=next_action:master.set_mode('LAND');next_action=time.monotonic()+2.
            elif rotation is not None and hold is not None and fresh:
                target=bounded_target(o[1],hold,1.2)
                navigating=mission_goal is not None and state in ('outbound','returning')
                if navigating:
                    path=plan.get('path',[])
                    valid=bool(path) and now-plan_at<2. and np.linalg.norm(np.asarray(plan.get('goal',hold))-hold)<.1
                    if valid:
                        recovery=plan.get('reason')=='recovering_clearance'
                        target=grid.steering_target(o[1],path,recovery=recovery) if grid is not None else None
                        if target is None:
                            hold=o[1].copy();pause_since=now;previous_state=state
                            transition('holding','planning_wait',now);target=hold
                    else:
                        hold=o[1].copy();pause_since=now;previous_state=state;transition('holding','planning_wait',now);target=hold
                if state in ('outbound','returning'):
                    clear=c is not None and now-c[0]<.5 and segment_clear(c[1],o[1],target,radius=.8 if plan.get('reason')=='recovering_clearance' else 2. if mission_goal is not None else .65)
                    if not clear:
                        hold=o[1].copy();pause_since=now;previous_state=state;transition('holding','obstacle_or_unknown_cloud',now);target=hold
                p=rotation@(target-anchor)+origin
                if mission_goal is not None:p[2]=origin[2]-height
                mask=position_mask;velocity=np.zeros(3)
                if mission_goal is not None and state in ('outbound','returning','holding'):
                    delta=target[:2]-o[1][:2];length=np.linalg.norm(delta)
                    limit=.5 if plan.get('reason')=='recovering_clearance' else mission_speed
                    desired=delta/max(length,1.e-9)*min(limit,.8*length) if state!='holding' else np.zeros(2)
                    dt=min(.1,max(.01,now-last_velocity_time)) if last_velocity_time is not None else .02
                    change=desired-last_velocity;last_velocity+=change*min(1.,.8*dt/max(np.linalg.norm(change),1.e-9));last_velocity_time=now
                    if state=='holding':last_velocity[:]=0.
                    elif grid is not None and plan.get('reason')!='recovering_clearance':
                        # Validate the braking corridor too, not just a distant
                        # target. Never retain velocity into newly blocked cells.
                        stopping=o[1]+np.r_[last_velocity,0.]*(np.linalg.norm(last_velocity)/1.6+.3)
                        if not grid.known_clear(o[1],stopping):last_velocity[:]=0.
                    velocity=rotation@np.r_[last_velocity,0.]
                    mask=(position_mask|mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE|mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE)&~(mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE|mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE)
                master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    mask,*p,*velocity,0,0,0,0,0)
            elif armed and not fresh and rotation is not None:
                # Brief callback skew or drop: stop horizontal motion while
                # preserving the OFFBOARD stream. Sustained loss lands above.
                mask=(position_mask|mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE|mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE)&~(mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE|mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE)
                master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    mask,0,0,origin[2]-height,0,0,0,0,0,0,0,0)
            if time.monotonic()-last_report>.2:
                report=dict(state=state,reason=reason,armed=armed,mode=mode,connected=time.monotonic()-heartbeat_at<3 and time.monotonic()-position_at<1.,
                    heartbeat_age_s=time.monotonic()-heartbeat_at if math.isfinite(heartbeat_at) else None,
                    position_ned_m=local.tolist() if local is not None else None,
                    relative_height_m=float(o[1][2]-anchor[2]) if o and anchor is not None else None,
                    external_pose_messages=ev_sent,lio_fresh=fresh,lio_age_s=now-o[0] if o else None,localization=loc,localization_age_s=now-loc_at if math.isfinite(loc_at) else None,
                    breadcrumbs=len(breadcrumbs),return_remaining=len(return_route),gnss_fusion_enabled=False,keys=[],velocity_ned_mps=measured_velocity.tolist() if measured_velocity is not None else None,
                    speed_mps=float(np.linalg.norm(measured_velocity)) if measured_velocity is not None else None,
                    barometric_relative_height_m=baro_height,height_mode='fixed_barometric',cruise_height_m=height,
                    goal_distance_horizontal_m=float(np.linalg.norm(o[1][:2]-mission_goal[:2])) if o and mission_goal is not None else None,
                    goal_radius_m=goal_radius,px4_messages=px4_messages[-10:],recent_events=events[-10:],
                    status_wall_time_s=time.time(),
                    return_path_lio=[p.tolist() for p in return_route],
                    mission_type='goal_exploration' if mission_goal is not None else 'recorded_return',goal_reached=goal_reached,
                    goal_lio=mission_goal.tolist() if mission_goal is not None else None,planner=plan,
                    sim_time_ns=int(now*1e9),error=failures[-1] if failures else None,autonomy=True)
                payload=json.dumps(report)
                for filename in ['navigation-status.json','teleop-status.json']:
                    tmp=run/(filename+'.tmp');tmp.write_text(payload+'\n');tmp.replace(run/filename)
                nav_pub.publish(String(data=payload));teleop_pub.publish(String(data=payload));last_report=time.monotonic()
            time.sleep(.02)
    finally:
        running=False;log.close()
        for sub in subscribers:sub.unregister()
        master.close()


if __name__=='__main__':main()
