"""Real-time rigid-body/sensor runner plus the unchanged full-stack backend."""
from collections import Counter
from contextlib import ExitStack
import heapq
import json
import math
from pathlib import Path
import time
import numpy as np
from .bullet_backend import BulletBackend, ReferenceController, rotation
from .physics_scene import load_world
from .physics_sensors import sensor_profile, RollingLidar, GnssModel, imu_sample
from .imu_model import ImuConfig, ImuNoiseModel
from .physics_web import LiveView
from .recording import RecordingWriter
from .schema import Event


def simulate(args):
    if args.backend=='gazebo':
        bad=[v for v in args._provided_simulation_options if v not in ('--backend','--root','--dry-run')]
        if bad:raise ValueError('Place original Gazebo arguments after --, for example: simulate --backend gazebo -- --web --scene simple')
        from .backends import run_gazebo
        extra=args.gazebo_args
        if extra and extra[0]=='--':extra=extra[1:]
        # Bullet flags are not translated into vaguely equivalent PX4 settings.
        return run_gazebo(extra,args.root,args.dry_run)
    if args.gazebo_args:raise ValueError('Trailing arguments are only accepted by the Gazebo compatibility backend')
    if args.output is None or args.world is None:raise ValueError('Bullet requires --output and --world')
    values=(args.duration_s,args.height_m,args.route_distance_m,args.real_time_factor,args.gps_delay_s,args.gps_rate_hz,args.serve_after_s,*args.spawn,*args.spoof_rate_enu)
    if not all(math.isfinite(v) for v in values) or not 0<args.duration_s<=3600 or not .1<=args.height_m<=100 or not 0<=args.route_distance_m<=1000:
        raise ValueError('Invalid duration, height, route or finite parameter')
    if not 0<=args.real_time_factor<=10 or not 0<=args.gps_delay_s<=5 or not 0<args.gps_rate_hz<=50 or not 0<=args.serve_after_s<=86400:
        raise ValueError('Invalid pacing/GPS/dwell parameter')
    if args.spoof_start_s!=math.inf and (not math.isfinite(args.spoof_start_s) or args.spoof_start_s<0):raise ValueError('Invalid spoof start time')
    if args.dry_run:return dict(backend='bullet',world=str(args.world),output=str(args.output),scope='experimental rigid body; not PX4/FAST-LIO')
    profile=sensor_profile(args.sensor_profile,args.lidar_model)
    scene=load_world(args.world,args.model_root);lidar=RollingLidar(profile['lidar'],.002)
    imu=ImuNoiseModel(ImuConfig.from_dict(profile['imu_ros']));gps_model=GnssModel(profile['gps'])
    output=Path(args.output)
    if output.exists():raise FileExistsError('Refusing to overwrite '+str(output))
    view=None;counts=Counter();pending=[];sequence=0;next_imu=next_gps=0
    last_gps=None;contact_steps=0;frames=0;interrupted=False;max_tracking=0.
    began=time.perf_counter();max_lag=0.;spoof_rate=np.array(args.spoof_rate_enu)
    attack_ns=None if args.spoof_start_s==math.inf else round(args.spoof_start_s*1e9)
    spawn=np.array(args.spawn,dtype=float);gps_period=round(1e9/args.gps_rate_hz);imu_period=round(1e9/profile['imu_ros']['rate_hz'])
    # State, sensors, collision queries and stepping all run on this one thread.
    with ExitStack() as resources, BulletBackend(scene,spawn=spawn) as backend:
        controller=ReferenceController(backend)
        if args.web:
            view=LiveView(args.port,scene,output);resources.callback(view.close);print('实时物理试验：'+view.url,flush=True)
        output.mkdir(parents=True,exist_ok=False)
        metadata=dict(source='Bullet real-time physics',backend='bullet',sensor_profile=profile,
                      antenna_offset_body_m=[.05,0,.04],labels=dict(attack_start_ns=attack_ns),
                      controller='diagnostic reference controller; not PX4',control_source=args.control_source,
                      scope='Physical sensor generation; no FAST-LIO, no autonomous obstacle planning',
                      lidar_time_unit='seconds from frame start',gps_delay_s=args.gps_delay_s,
                      gravity_mps2=scene['gravity'],step_s=backend.dt)
        (output/'sensor_profile.json').write_text(json.dumps(profile,indent=2)+'\n',encoding='utf-8')
        (output/'vehicle.json').write_text(json.dumps(backend.cfg,indent=2)+'\n',encoding='utf-8')
        (output/'scene.json').write_text(json.dumps(scene,indent=2)+'\n',encoding='utf-8')
        def enqueue(event):
            nonlocal sequence
            sequence+=1;heapq.heappush(pending,(event.arrival_ns,sequence,event))
        with RecordingWriter(output/'recording',metadata) as writer,(output/'physics.jsonl').open('w',encoding='utf-8') as log:
            began=time.perf_counter()
            try:
                for tick in range(math.ceil(args.duration_s/backend.dt)+1):
                    state=backend.state();now=state['stamp_ns'];t=now/1e9
                    # Route is a diagnostic target, not an obstacle-aware planner.
                    north=min(args.route_distance_m,max(0,t-5)*1.2) if args.flight_pattern=='route' else 0.
                    target=spawn+np.array([0.,north,args.height_m])
                    offset=spoof_rate*max(0,t-args.spoof_start_s) if attack_ns is not None else np.zeros(3)
                    if now>=next_imu:
                        sample=imu_sample(imu,state,backend.gravity)
                        enqueue(Event(now,now,'imu','imu','body_flu',dict(acceleration_mps2=list(map(float,sample['linear_acceleration'])),
                            angular_velocity_radps=list(map(float,sample['angular_velocity'])),orientation_xyzw=list(map(float,sample['orientation'])),
                            acceleration_variance=sample['accel_variance'],angular_velocity_variance=sample['gyro_variance'])))
                        next_imu+=imu_period
                    if now>=next_gps:
                        antenna=state['position']+rotation(state['orientation'])@np.array([.05,0,.04])
                        normal=gps_model.sample(now,antenna);final=normal+offset
                        arrival=now+round(args.gps_delay_s*1e9)
                        antenna_velocity=state['velocity']+rotation(state['orientation'])@np.cross(state['angular_velocity'],[.05,0,.04])
                        for name,pos in [('gnss_normal',normal),('gnss',final)]:
                            velocity=antenna_velocity.copy()
                            if name=='gnss' and attack_ns is not None and now>=attack_ns and profile['gps'].get('spoof_velocity_consistent',True):velocity+=spoof_rate
                            enqueue(Event(now,arrival,'gnss',name,'enu',dict(position_m=pos.tolist(),point='antenna',
                                velocity_mps=velocity.tolist(),variance_enu_m2=profile['gps']['reported_variance_enu_m2'])))
                        next_gps+=gps_period
                    if tick%5==0:
                        enqueue(Event(now,now,'pose','truth','enu',dict(position_m=state['position'].tolist(),point='body',
                            orientation_xyzw=state['orientation'].tolist(),velocity_mps=state['velocity'].tolist())))
                    cloud=lidar.update(backend)
                    while pending and pending[0][0]<=now:
                        _,_,event=heapq.heappop(pending);writer.write(event);counts[event.stream]+=1
                        if event.stream=='gnss':last_gps=event
                    if cloud is not None:
                        writer.cloud(cloud['stamp_ns'],now,'lidar','lidar',cloud['fields']);counts['lidar']+=1;frames+=1
                        if view:
                            points=cloud['world_points'];stride=max(1,math.ceil(len(points)/8000))
                            view.update(cloud=points[::stride].astype(float).tolist(),lidar_frames=frames)
                    observations={}
                    if args.control_source=='gnss':
                        if last_gps is None:motors=np.zeros(4)
                        else:
                            age=(now-last_gps.stamp_ns)/1e9;v=np.array(last_gps.data['velocity_mps'])
                            position=np.array(last_gps.data['position_m'])+age*v-rotation(state['orientation'])@np.array([.05,0,.04])
                            observations=dict(observed_position=position,observed_velocity=v)
                            motors=controller.motors(target,**observations)
                    else:motors=controller.motors(target)
                    contacts=backend.contacts();contact_steps+=bool(contacts)
                    tracking=float(np.linalg.norm(state['position']-target));max_tracking=max(max_tracking,tracking)
                    if tick%25==0:
                        row=dict(stamp_ns=now,position_m=state['position'].tolist(),velocity_mps=state['velocity'].tolist(),
                            target_m=target.tolist(),motor_radps=motors.tolist(),contacts=contacts,spoof_offset_m=offset.tolist())
                        log.write(json.dumps(row,allow_nan=False)+'\n');log.flush()
                        if view:view.update(phase='running',sim_time_s=t,position=state['position'].tolist(),velocity=state['velocity'].tolist(),
                            target=target.tolist(),gps=None if last_gps is None else last_gps.data['position_m'],contacts=len(contacts),
                            spoof_offset=offset.tolist(),control_source=args.control_source,real_time_factor=t/max(1e-9,time.perf_counter()-began))
                    if tick==math.ceil(args.duration_s/backend.dt):break
                    backend.step(motors)
                    if args.real_time_factor:
                        due=began+(tick+1)*backend.dt/args.real_time_factor;delay=due-time.perf_counter()
                        if delay>0:time.sleep(delay)
                        else:max_lag=max(max_lag,-delay)
            except KeyboardInterrupt:
                interrupted=True
            # Keep true sample stamps for pending GNSS. Availability can extend
            # beyond the final physical tick; no future truth is synthesized.
            while pending:
                _,_,event=heapq.heappop(pending);writer.write(event);counts[event.stream]+=1
        result=dict(backend='bullet',status='interrupted_saved' if interrupted else 'completed',
            simulated_duration_s=backend.state()['stamp_ns']/1e9,wall_time_s=time.perf_counter()-began,
            counts=dict(counts),lidar_frames=frames,contact_steps=int(contact_steps),max_pacing_lag_s=max_lag,
            final_position_enu_m=backend.state()['position'].tolist(),final_target_enu_m=target.tolist(),
            final_tracking_error_m=float(np.linalg.norm(backend.state()['position']-target)),
            max_tracking_error_m=max_tracking,reference_control_source=args.control_source,
            scope='Experimental physical sensor backend. Not PX4/FAST-LIO or Gazebo-equivalent navigation.')
        (output/'simulation-summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        from .replay import replay
        replay(output/'recording',output/'evaluation')
        if view:
            view.update(phase=result['status'])
            try:
                if args.serve_after_s and not interrupted:
                    print(f'运行已保存，预览保留 {args.serve_after_s:g} 秒：{view.url}',flush=True)
                    deadline=time.monotonic()+args.serve_after_s
                    while time.monotonic()<deadline:time.sleep(max(0.,min(.2,deadline-time.monotonic())))
            except KeyboardInterrupt:pass
        return result
