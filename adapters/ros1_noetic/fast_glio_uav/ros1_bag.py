#!/usr/bin/env python3
"""Capture FAST_GLIO sensors and merge SignalSim GNSS into a ROS 1 bag."""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np

import rosbag
import rosgraph
import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rtklib.msg import GNSS_Info, GNSS_Info_ZD
from std_msgs.msg import String
from scene_profile import from_environment, load_scene_profile
from teleop_controls import trajectory_window
from capture_health import raise_if_simulation_crashed,raise_recorder_exit,require_recording_space
from run_storage import archive_runtime

HERE = Path(__file__).resolve().parent
TOPICS = ['/clock', '/ground_truth/state', '/imu/data', '/velodyne_points']
LEVER = [0.005, -0.05, 0.15]
ORIGIN = from_environment()['origin_lla_deg_m']
C = 299792458.0


def ns(stamp):
    return stamp.secs * 1_000_000_000 + stamp.nsecs


def ros_time(stamp):
    return rospy.Time(stamp // 1_000_000_000, stamp % 1_000_000_000)


def stop(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


def capture(args):
    run = args.run
    require_recording_space(run.parent)
    run.mkdir(parents=True, exist_ok=False)
    scene = from_environment()
    (run/'scene-profile.json').write_text(json.dumps(scene, indent=2)+'\n')
    if scene.get('asset_manifest'):
        (run/'scene-assets.json').write_bytes(Path(scene['asset_manifest']).read_bytes())
    sensor_profile = os.environ.get('MSIM_SENSOR_PROFILE')
    if sensor_profile:
        (run / 'sensor_profile.json').write_bytes(Path(sensor_profile).read_bytes())
        profile=json.loads(Path(sensor_profile).read_text())
        pattern=profile['lidar'].get('pattern_file')
        if pattern:
            (run/'lidar-pattern.csv').write_bytes(Path(pattern).read_bytes())
            provenance=Path(pattern).with_name('source.json')
            if provenance.is_file():
                (run/'lidar-pattern-source.json').write_bytes(provenance.read_bytes())
            for source_name, archive_name in (('LICENSE','lidar-pattern-LICENSE'),('pattern-summary.json','lidar-pattern-summary.json')):
                extra=Path(pattern).with_name(source_name)
                if extra.is_file():
                    (run/archive_name).write_bytes(extra.read_bytes())
    plugin_dir = HERE.parents[2] / 'build/gazebo-sensors'
    plugin_hashes = {}
    for name in ('libmsim_gps_relay.so', 'libmsim_scanning_lidar.so'):
        library = plugin_dir / name
        if library.is_file():
            plugin_hashes[name] = hashlib.sha256(library.read_bytes()).hexdigest()
    if plugin_hashes:
        (run / 'sensor-plugins.json').write_text(json.dumps(plugin_hashes, indent=2)+'\n')
    for variable, filename in (('MSIM_WORLD_SOURCE','world.sdf'),
                               ('MSIM_MODEL_SOURCE','vehicle.sdf')):
        if os.environ.get(variable):
            (run / filename).write_bytes(Path(os.environ[variable]).read_bytes())
    if args.fastglio_config:
        (run/'algorithm.yaml').write_bytes(args.fastglio_config.read_bytes())
    if args.fastglio_launch:
        (run/'algorithm.launch').write_bytes(args.fastglio_launch.read_bytes())
    if args.fastglio_rviz_config:
        (run/'algorithm.rviz').write_bytes(args.fastglio_rviz_config.read_bytes())
    (run/'capture-settings.json').write_text(json.dumps(dict(
        duration_s=args.duration_s,route_clock=os.environ.get('MSIM_ROUTE_CLOCK','wall'),
        interactive=args.interactive, mapping_only=args.mapping_only,
        autonomy=os.environ.get('MSIM_AUTONOMY')=='1',
        localization_map=os.environ.get('MSIM_LOCALIZATION_MAP') or None,
        goal_relative_enu_m=[float(x) for x in os.environ['MSIM_GOAL_ENU'].split(',')] if os.environ.get('MSIM_GOAL_ENU') else None,
        mission_speed_mps=float(os.environ.get('MSIM_MISSION_SPEED_MPS','2')) if os.environ.get('MSIM_GOAL_ENU') else None,
        goal_radius_m=float(os.environ.get('MSIM_GOAL_RADIUS_M','2')) if os.environ.get('MSIM_GOAL_ENU') else None,
        autonomy_timeout_s=float(os.environ.get('MSIM_AUTONOMY_TIMEOUT_S','150')) if os.environ.get('MSIM_AUTONOMY')=='1' else None,
        teleop_takeoff_height_m=float(os.environ.get('MSIM_TELEOP_HEIGHT_M','5')) if args.interactive else None,
        teleop_speed_mps=float(os.environ.get('MSIM_TELEOP_SPEED_MPS','2')) if args.interactive else None,
        flight_pattern=os.environ.get('MSIM_PX4_FLIGHT_PATTERN'),
        frontend=os.environ.get('MSIM_FASTGLIO_FRONTEND','marsim'),
        gnss_source=os.environ.get('MSIM_GNSS_SOURCE','legacy'),
        point_filter_num=args.fastglio_point_filter_num,
        fusion_processing_delay_s=float(os.environ.get('MSIM_FUSION_DELAY_S', '0')),
        world_source=os.environ.get('MSIM_WORLD_SOURCE'),
        scene_key=os.environ.get('MSIM_SCENE_KEY',scene['name']),
        origin_lla_deg_m=scene['origin_lla_deg_m'],
        spawn_enu_m=scene['spawn_enu_m'], flight_height_m=scene['flight_height_m'],
        spoof_start_s=os.environ.get('MSIM_SPOOF_START_S','inf'),
        spoof_east_rate_mps=os.environ.get('MSIM_SPOOF_EAST_RATE_MPS','0')),
        indent=2)+'\n')
    if os.environ.get('MSIM_PX4_FLIGHT_PATTERN') == 'route':
        route_copy = run / 'route_plan.json'
        route_copy.write_bytes(Path(os.environ.get('MSIM_ROUTE_PLAN',str(HERE / 'route_plan.json'))).read_bytes())
        os.environ['MSIM_ROUTE_PLAN'] = str(route_copy)
    Path(os.environ.get('ROS_LOG_DIR', str(run / 'roslogs'))).mkdir(parents=True, exist_ok=True)
    launch = record = controller = injector = visualizer = fastglio = mapper = web_console = None
    state = dict(clock=None, counts=Counter(), truth=None)
    stop_requested = dict(value=False, reason=None)
    stop_file = run/'capture-stop.json'
    previous_sigint = signal.getsignal(signal.SIGINT)
    def request_stop(*_):
        if record is None:
            raise KeyboardInterrupt
        stop_requested.update(value=True, reason='keyboard_interrupt')
    if args.interactive:
        signal.signal(signal.SIGINT, request_stop)
    launch_log = (run / 'launch.log').open('w')
    record_log = (run / 'record.log').open('w')
    controller_log = (run / 'controller.log').open('w')
    injector_log = (run / 'injector.log').open('w')
    visualizer_log = (run / 'gazebo-viz.log').open('w')
    fastglio_log = (run / 'fastglio-live.log').open('w')
    tile_log = (run / 'window-layout.log').open('w')
    map_log = (run / 'mapping.log').open('w')
    web_log = (run / 'web-console.log').open('w')
    phase='starting_ros'
    try:
        launch_file = args.launch_file or HERE / 'uav_world.launch'
        launch_args = (args.launch_arg if args.launch_file else
                       ['flight_mode:=' + args.flight_mode])
        launch = subprocess.Popen(['roslaunch', str(launch_file),
            'gui:=' + ('true' if args.gui else 'false'), *launch_args],
            stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 90
        master = rosgraph.Master('/msim_fastglio_capture')
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise RuntimeError('roslaunch exited; see launch.log')
            try:
                master.getPid()
                break
            except Exception:
                time.sleep(0.2)
        else:
            raise RuntimeError('ROS master did not start')
        rospy.init_node('msim_fastglio_capture', anonymous=True, disable_signals=True)

        def save_overview(message):
            path=run/'gazebo-scene-overview.png'
            if path.exists():return
            from PIL import Image
            mode={'rgb8':'RGB','bgr8':'BGR'}.get(message.encoding)
            if mode:
                Image.frombytes('RGB',(message.width,message.height),bytes(message.data),
                                'raw',mode,message.step).save(path)
        from sensor_msgs.msg import Image as RosImage
        overview_sub=rospy.Subscriber('/msim/scene_overview/image_raw',RosImage,
                                     save_overview,queue_size=1,buff_size=8*1024*1024)

        def on_clock(msg):
            state['clock'] = ns(msg.clock)
            state['counts']['clock'] += 1
        def on_truth(msg):
            state['truth'] = ns(msg.header.stamp)
            state['counts']['truth'] += 1
        def on_imu(_):
            state['counts']['imu'] += 1
        def on_lidar(_):
            state['counts']['lidar'] += 1
        subs = [rospy.Subscriber('/clock', Clock, on_clock, queue_size=10),
                rospy.Subscriber('/ground_truth/state', Odometry, on_truth, queue_size=100),
                rospy.Subscriber('/imu/data', rospy.AnyMsg, on_imu, queue_size=100),
                rospy.Subscriber('/velodyne_points', rospy.AnyMsg, on_lidar, queue_size=20)]
        ready_deadline = time.monotonic() + 90
        phase='waiting_sensors'
        while time.monotonic() < ready_deadline and not all(state['counts'][x] for x in
                                                            ('clock', 'truth', 'imu', 'lidar')):
            raise_if_simulation_crashed(run/'launch.log')
            if launch.poll() is not None:
                raise RuntimeError('Gazebo exited; see launch.log')
            time.sleep(0.1)
        if not all(state['counts'][x] for x in ('clock', 'truth', 'imu', 'lidar')):
            raise RuntimeError('required sensor topics did not arrive')
        if args.injector_script:
            phase='waiting_gnss'
            env = dict(os.environ, MSIM_GNSS_LOG=str(run / 'injected-gps.jsonl'))
            injector = subprocess.Popen([sys.executable, str(args.injector_script)],
                stdout=injector_log, stderr=subprocess.STDOUT,
                env=env, start_new_session=True)
            injection_deadline = time.monotonic() + 45
            while time.monotonic() < injection_deadline:
                raise_if_simulation_crashed(run/'launch.log')
                if injector.poll() is not None:
                    raise RuntimeError('online GNSS injector exited; see injector.log')
                if (run / 'injected-gps.jsonl').is_file() and \
                        (run / 'injected-gps.jsonl').stat().st_size > 0:
                    break
                time.sleep(0.2)
            else:
                raise RuntimeError('online GNSS injector not publishing; see injector.log')
        if args.gazebo_viz_script:
            ready = run / 'gazebo-viz-ready.json'
            env = dict(os.environ, MSIM_GAZEBO_VIZ_READY=str(ready))
            visualizer = subprocess.Popen([sys.executable, str(args.gazebo_viz_script)],
                stdout=visualizer_log, stderr=subprocess.STDOUT,
                env=env, start_new_session=True)
            viz_deadline = time.monotonic() + 45
            while time.monotonic() < viz_deadline:
                if visualizer.poll() is not None:
                    raise RuntimeError('Gazebo visualizer exited; see gazebo-viz.log')
                if ready.is_file():
                    break
                time.sleep(0.2)
            else:
                raise RuntimeError('Gazebo visualizer did not start; see gazebo-viz.log')
        if args.fastglio_launch:
            phase='starting_fastglio'
            algorithm_args = [
                'roslaunch', str(args.fastglio_launch),
                'config_file:=' + str(args.fastglio_config),
                'rviz_config:=' + str(args.fastglio_rviz_config),
                'rviz:=' + ('true' if args.gui else 'false'),
                'point_filter_num:=' + str(args.fastglio_point_filter_num)]
            if args.mapping_only:
                algorithm_args.append('mapping_only:=true')
            fastglio = subprocess.Popen(algorithm_args,
                stdout=fastglio_log, stderr=subprocess.STDOUT,
                start_new_session=True)
            fastglio_deadline = time.monotonic() + 45
            while time.monotonic() < fastglio_deadline:
                raise_if_simulation_crashed(run/'launch.log')
                if fastglio.poll() is not None:
                    raise RuntimeError('FAST_GLIO exited; see fastglio-live.log')
                try:
                    subscribers = master.getSystemState()[1]
                    if any('/laserMapping' in nodes for _, nodes in subscribers):
                        break
                except (IndexError, KeyError, TypeError, rosgraph.MasterError):
                    pass
                time.sleep(0.2)
            else:
                raise RuntimeError('FAST_GLIO did not subscribe to sensors; see fastglio-live.log')
            subprocess.run(['rosparam','dump',str(run/'parameters.yaml')],check=True)
        if args.map_script:
            mapper = subprocess.Popen([sys.executable, str(args.map_script)],
                stdout=map_log, stderr=subprocess.STDOUT, start_new_session=True)
        if args.web_console_script:
            web_console = subprocess.Popen([sys.executable, str(args.web_console_script),
                '--run', str(run), '--port', str(args.web_console_port), '--ros'],
                stdout=web_log, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(0.5)
            if web_console.poll() is not None:
                raise RuntimeError('Web console exited; see web-console.log')
        if args.tile_windows_script:
            tiled = subprocess.run([sys.executable, str(args.tile_windows_script)],
                                   stdout=tile_log, stderr=subprocess.STDOUT,
                                   timeout=45, check=False)
            if tiled.returncode:
                raise RuntimeError('Gazebo/RViz window layout failed; see window-layout.log')
        record_topics = [*TOPICS]
        if args.injector_script:
            record_topics.extend(['/gnss/spoof_detection', '/msim/spoofed_gnss_odom',
                                  '/msim/normal_gnss_odom', '/msim/gps_samples', '/msim/gps_attack'])
        record_topics.append('/msim/lidar_scan_info')
        record_topics.extend(['/livox/lidar','/msim/livox_lidar_for_fusion'])
        if args.fastglio_launch:
            record_topics.extend(['/Odometry', '/path', '/loose_odom', '/loose_path',
                                  '/rtk_path', '/gnss_loose/diagnostics'])
            record_topics.append('/msim/velodyne_points_annotated')
        if args.interactive:
            record_topics.extend(['/msim/teleop/command', '/msim/teleop/status',
                                  '/cloud_registered', '/msim/lio_map/status'])
            if os.environ.get('MSIM_AUTONOMY')=='1':
                record_topics.extend(['/msim/navigation/status','/msim/return_path','/msim/localized_odom','/msim/planned_path'])
        record = subprocess.Popen(['rosbag', 'record', '-O', str(run / 'sensors.bag'), *record_topics],
            stdout=record_log, stderr=subprocess.STDOUT, start_new_session=True)
        if args.controller_script:
            controller = subprocess.Popen([sys.executable, str(args.controller_script)],
                stdout=controller_log, stderr=subprocess.STDOUT, start_new_session=True)
        start = state['clock']
        deadline = time.monotonic() + max(120, args.duration_s * 8)
        next_report = 10
        phase='recording'
        while args.duration_s == 0 or (state['clock'] or 0) - start < (args.duration_s + 2) * 1_000_000_000:
            raise_if_simulation_crashed(run/'launch.log')
            if args.interactive and (stop_requested['value'] or stop_file.exists()):
                break
            if launch.poll() is not None:
                raise RuntimeError('仿真启动器退出；请检查 launch.log')
            if record.poll() is not None:
                raise_recorder_exit(run/'record.log',record.returncode)
            if controller is not None and controller.poll() is not None:
                raise RuntimeError('flight controller exited; see controller.log')
            if injector is not None and injector.poll() is not None:
                raise RuntimeError('online GNSS injector exited; see injector.log')
            if visualizer is not None and visualizer.poll() is not None:
                raise RuntimeError('Gazebo visualizer exited; see gazebo-viz.log')
            if fastglio is not None and fastglio.poll() is not None:
                if args.interactive:
                    stop_requested.update(value=True, reason='mapping_window_closed')
                    break
                raise RuntimeError('FAST_GLIO exited; see fastglio-live.log')
            if mapper is not None and mapper.poll() is not None:
                raise RuntimeError('map recorder exited; see mapping.log')
            if web_console is not None and web_console.poll() is not None:
                raise RuntimeError('web console exited; see web-console.log')
            if not args.interactive and time.monotonic() > deadline:
                raise RuntimeError('simulation clock stalled')
            elapsed = ((state['clock'] or start) - start) / 1e9
            if elapsed >= next_report:
                limit = str(args.duration_s) if args.duration_s else '手动结束'
                print(f'仿真 {elapsed:.0f} / {limit} 秒', flush=True)
                next_report += 10
            time.sleep(0.1)
        for sub in subs:
            sub.unregister()
    except BaseException as error:
        (run/'capture-failure.json').write_text(json.dumps(dict(phase=phase,error=str(error),
            error_type=type(error).__name__,last_sim_time_ns=state['clock'],
            sensor_counts=dict(state['counts']),launch_log=str(run/'launch.log')),indent=2)+'\n')
        raise
    finally:
        if args.interactive:
            stop(controller)
        stop(record)
        if not args.interactive:
            stop(controller)
        stop(fastglio)
        stop(mapper)
        stop(web_console)
        stop(visualizer)
        stop(injector)
        stop(launch)
        if os.environ.get('MSIM_RUNTIME_DIR'):
            try:archive_runtime(os.environ['MSIM_RUNTIME_DIR'],run)
            except (OSError,ValueError) as error:print(f'运行日志归档失败，原件保留在 {os.environ["MSIM_RUNTIME_DIR"]}: {error}',file=sys.stderr)
        launch_log.close()
        record_log.close()
        controller_log.close()
        injector_log.close()
        visualizer_log.close()
        fastglio_log.close()
        tile_log.close()
        map_log.close()
        web_log.close()
        if args.interactive:
            signal.signal(signal.SIGINT, previous_sigint)
    bag = run / 'sensors.bag'
    if not bag.is_file() or bag.stat().st_size == 0:
        raise RuntimeError('sensors.bag missing or empty')
    if args.interactive:
        reason = stop_requested['reason'] or ('teleop_finish' if stop_file.exists() else 'duration_limit')
        autonomous=os.environ.get('MSIM_AUTONOMY')=='1'
        if autonomous and stop_file.exists():
            reason=json.loads(stop_file.read_text()).get('reason',reason)
        (run/'capture-session.json').write_text(json.dumps(dict(
            mode='autonomy' if autonomous else 'teleop', stop_reason=reason, requested_duration_s=args.duration_s,
            recording_complete=True, raw_bag=str(bag)), indent=2)+'\n')
    if args.fastglio_launch:
        required = (('/Odometry', '/path', '/cloud_registered') if args.mapping_only else
                    ('/msim/spoofed_gnss_odom', '/Odometry', '/path',
                    '/loose_odom', '/loose_path', '/rtk_path',
                    '/gnss_loose/diagnostics'))
        with rosbag.Bag(str(bag)) as recorded:
            info = recorded.get_type_and_topic_info().topics
            counts = {topic: info[topic].message_count if topic in info else 0
                      for topic in required}
            diagnostics = [msg.data for _, msg, _ in recorded.read_messages(
                topics=['/gnss_loose/diagnostics'])]
        accepted = sum(len(row) > 2 and int(row[1]) == 4 and row[2] == 1.0
                       for row in diagnostics)
        report = dict(topic_counts=counts,
                      gnss_alignment_reached=any(len(row) > 2 and row[2] == 1.0
                                                 for row in diagnostics),
                      gnss_updates_accepted=accepted)
        report['pass_live_loose_gate'] = (all(counts.values()) and
                                          counts.get('/loose_odom',0) >= 10 and
                                          accepted >= 5)
        if args.mapping_only:
            report = dict(mode='lidar_imu_mapping', topic_counts=counts,
                          pass_mapping_gate=bool(all(counts.values()) and counts['/Odometry'] >= 10))
        (run / 'fastglio-live-report.json').write_text(
            json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        if args.mapping_only and not report['pass_mapping_gate']:
            print('手动采集时间过短或建图数据不足；原始 bag 已保留。', flush=True)
        elif not args.mapping_only and not report['pass_live_loose_gate']:
            raise RuntimeError('FAST_GLIO loose coupling did not produce accepted updates')
    print(str(bag), flush=True)


def qmul(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def qinv(q):
    return [-q[0], -q[1], -q[2], q[3]]


def ecef_basis(origin):
    lat, lon = map(math.radians, origin[:2])
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    return [[-so, -sl*co, cl*co], [co, -sl*so, cl*so], [0, cl, sl]]


def earth_q(matrix):
    # Matrix maps ENU vectors to ECEF vectors.
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        return [(m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s,
                (m[1][0]-m[0][1])/s, s/4]
    k = max(range(3), key=lambda i: m[i][i])
    if k == 0:
        s = math.sqrt(1+m[0][0]-m[1][1]-m[2][2])*2
        return [s/4, (m[0][1]+m[1][0])/s, (m[0][2]+m[2][0])/s,
                (m[2][1]-m[1][2])/s]
    if k == 1:
        s = math.sqrt(1+m[1][1]-m[0][0]-m[2][2])*2
        return [(m[0][1]+m[1][0])/s, s/4, (m[1][2]+m[2][1])/s,
                (m[0][2]-m[2][0])/s]
    s = math.sqrt(1+m[2][2]-m[0][0]-m[1][1])*2
    return [(m[0][2]+m[2][0])/s, (m[1][2]+m[2][1])/s, s/4,
            (m[1][0]-m[0][1])/s]


def vec(matrix, values):
    return [sum(row[i]*values[i] for i in range(3)) for row in matrix]


def geodetic_ecef(origin):
    lat, lon = map(math.radians, origin[:2])
    h = origin[2]
    a, e2 = 6378137.0, 6.69437999014e-3
    n = a / math.sqrt(1-e2*math.sin(lat)**2)
    return [(n+h)*math.cos(lat)*math.cos(lon),
            (n+h)*math.cos(lat)*math.sin(lon), (n*(1-e2)+h)*math.sin(lat)]


def trajectory(args):
    basis = ecef_basis(ORIGIN)
    origin = geodetic_ecef(ORIGIN)
    eq = earth_q(basis)
    samples = []
    counts = Counter()
    first = last = None
    interpolated = 0
    max_raw_gap_ns = 0
    with rosbag.Bag(str(args.run / 'sensors.bag')) as bag:
        for topic, msg, _ in bag.read_messages(topics=TOPICS):
            counts[topic] += 1
            if topic != '/ground_truth/state':
                continue
            stamp = ns(msg.header.stamp)
            if stamp <= 0:
                continue
            if samples and stamp <= samples[-1]['sim_time_ns']:
                if stamp == samples[-1]['sim_time_ns']:
                    continue
                raise ValueError('truth time went backward')
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            enu = [p.x, p.y, p.z]
            position = vec(basis, enu)
            quat = [q.x, q.y, q.z, q.w]
            norm = math.sqrt(sum(x*x for x in quat))
            if norm < 0.99:
                raise ValueError('invalid truth quaternion')
            quat = [x/norm for x in quat]
            sample = dict(sim_time_ns=stamp,
                position_ecef_m=[origin[i]+position[i] for i in range(3)],
                velocity_ecef_mps=[0.0]*3,
                orientation_earth_base_xyzw=qmul(eq, quat),
                angular_velocity_body_radps=[0.0]*3)
            if samples:
                prior = samples[-1]
                raw_gap = stamp-prior['sim_time_ns']
                max_raw_gap_ns = max(max_raw_gap_ns, raw_gap)
                if raw_gap > 20_000_000:
                    if raw_gap > 200_000_000:
                        raise ValueError(f'truth gap {raw_gap/1e9:.3f}s exceeds 200 ms repair limit')
                    pieces = math.ceil(raw_gap/10_000_000)
                    left_q = prior['orientation_earth_base_xyzw']
                    right_q = sample['orientation_earth_base_xyzw']
                    if sum(x*y for x,y in zip(left_q,right_q)) < 0:
                        right_q = [-x for x in right_q]
                    for j in range(1, pieces):
                        weight = j/pieces
                        qmid = [(1-weight)*x+weight*y for x,y in zip(left_q,right_q)]
                        qnorm = math.sqrt(sum(x*x for x in qmid))
                        middle = dict(sim_time_ns=prior['sim_time_ns']+round(raw_gap*weight),
                            position_ecef_m=[(1-weight)*x+weight*y for x,y in
                                zip(prior['position_ecef_m'],sample['position_ecef_m'])],
                            velocity_ecef_mps=[0.0]*3,
                            orientation_earth_base_xyzw=[x/qnorm for x in qmid],
                            angular_velocity_body_radps=[0.0]*3)
                        samples.append(middle)
                        interpolated += 1
                    prior = samples[-1]
                dt = (stamp-prior['sim_time_ns'])/1e9
                if dt > 0.0200001:
                    raise ValueError(f'truth gap {dt:.3f}s exceeds 20 ms')
                velocity = [(sample['position_ecef_m'][i]-prior['position_ecef_m'][i])/dt
                            for i in range(3)]
                delta = qmul(qinv(prior['orientation_earth_base_xyzw']),
                             sample['orientation_earth_base_xyzw'])
                if delta[3] < 0:
                    delta = [-x for x in delta]
                omega = [2*x/dt for x in delta[:3]]
                sample['velocity_ecef_mps'] = velocity
                sample['angular_velocity_body_radps'] = omega
                if len(samples) == 1:
                    prior['velocity_ecef_mps'] = velocity
                    prior['angular_velocity_body_radps'] = omega
            samples.append(sample)
            first = stamp if first is None else first
            last = stamp
    if len(samples) < 2 or any(counts[x] == 0 for x in TOPICS):
        raise ValueError('missing truth or sensor stream')
    if interpolated > len(samples)//100:
        raise ValueError('more than 1% of trajectory truth required interpolation')
    # Fill derivatives for inserted samples using the surrounding trajectory.
    for i in range(1, len(samples)-1):
        if samples[i]['velocity_ecef_mps'] != [0.0]*3:
            continue
        left, right = samples[i-1], samples[i+1]
        dt = (right['sim_time_ns']-left['sim_time_ns'])/1e9
        samples[i]['velocity_ecef_mps'] = [(right['position_ecef_m'][k]-left['position_ecef_m'][k])/dt
                                           for k in range(3)]
        delta = qmul(qinv(left['orientation_earth_base_xyzw']),
                     right['orientation_earth_base_xyzw'])
        if delta[3] < 0:
            delta = [-x for x in delta]
        samples[i]['angular_velocity_body_radps'] = [2*x/dt for x in delta[:3]]
    samples[0]['velocity_ecef_mps'] = samples[1]['velocity_ecef_mps']
    samples[0]['angular_velocity_body_radps'] = samples[1]['angular_velocity_body_radps']
    # Choose the first full 0.2 s epoch after a 1 s warmup; keep epochs inside bag.
    start = ((first + 1_000_000_000 + 199_999_999)//200_000_000)*200_000_000
    if args.duration_s == 0:
        start, args.duration_s = trajectory_window(first, last)
    if last < start + args.duration_s*1_000_000_000:
        raise ValueError('recorded truth does not cover requested duration')
    output = dict(schema_version=1, frame='map', antenna_offset_base_m=args.antenna_offset,
                  samples=samples)
    (args.run / 'trajectory.json').write_text(json.dumps(output, separators=(',', ':'))+'\n')
    report = dict(origin_lla_deg_m=ORIGIN, first_truth_ns=first,
                  last_truth_ns=last, gnss_origin_sim_time_ns=start,
                  duration_s=args.duration_s, sensor_counts=dict(counts),
                  interpolated_truth_samples=interpolated,
                  max_raw_truth_gap_ms=max_raw_gap_ns/1e6)
    (args.run / 'capture-report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


def earth_rotate(vector, angle):
    v = np.asarray(vector, dtype=float)
    return np.array([math.cos(angle)*v[0]+math.sin(angle)*v[1],
                     -math.sin(angle)*v[0]+math.cos(angle)*v[1], v[2]])


def receive_frame_satellite(satellite, receiver):
    sat = np.asarray(satellite, dtype=float)
    rec = np.asarray(receiver, dtype=float)
    transit = np.linalg.norm(sat-rec)/C
    return earth_rotate(sat, 7.2921151467e-5*transit)


def geometric_range(satellite, receiver):
    corrected = receive_frame_satellite(satellite, receiver)
    rec = np.asarray(receiver, dtype=float)
    distance = np.linalg.norm(corrected-rec)
    return distance, (rec-corrected)/distance


def differential_position(rover, base, base_position):
    """Solve code-DGNSS from common L1 satellites, without using UAV truth."""
    def usable(obs):
        return (obs.get('system', 'GPS') == 'GPS' and obs['code'] == '1C' and
                not obs.get('multipath_nlos', False) and
                abs(obs.get('multipath_code_bias_m', 0.0)) < 3.0)
    base_l1 = {o['prn']: o for o in base['observations'] if usable(o)}
    common = [o for o in rover['observations'] if usable(o) and o['prn'] in base_l1]
    if len(common) < 4:
        return None
    base_position = np.asarray(base_position, dtype=float)
    position = np.asarray(geodetic_ecef(ORIGIN), dtype=float)
    bias_m = 0.0
    for _ in range(10):
        rows, residuals = [], []
        for obs in common:
            reference = base_l1[obs['prn']]
            base_range, _ = geometric_range(reference['sat_pos'], base_position)
            base_residual = reference['pseudorange_m'] - (base_range-C*reference['sat_clock_bias_s'])
            corrected_code = obs['pseudorange_m']-base_residual
            rover_range, derivative = geometric_range(obs['sat_pos'], position)
            predicted = rover_range-C*obs['sat_clock_bias_s']+bias_m
            rows.append([*derivative, 1.0])
            residuals.append(corrected_code-predicted)
        matrix = np.asarray(rows)
        if np.linalg.matrix_rank(matrix) < 4 or np.linalg.cond(matrix) > 100:
            return None
        update = np.linalg.lstsq(matrix, np.asarray(residuals), rcond=None)[0]
        position += update[:3]
        bias_m += update[3]
        if np.linalg.norm(update[:3]) < 1e-4:
            break
    if not np.isfinite(position).all() or np.linalg.norm(position-base_position) > 10000:
        return None
    rms = float(np.sqrt(np.mean(np.square(residuals))))
    if rms > 20:
        return None
    covariance = np.linalg.inv(matrix.T @ matrix)
    pdop = float(np.sqrt(np.trace(covariance[:3, :3])))
    if pdop > 10:
        return None
    return dict(position=position.tolist(), satellites=len(common), rms_m=rms,
                pdop=pdop, clock_bias_m=float(bias_m))


def raw_message(epoch, stamp, solution):
    info = GNSS_Info()
    info.header.stamp = ros_time(stamp)
    info.header.frame_id = 'gnss_antenna'
    info.gpsWeek = epoch['gps_week']
    info.weekSec = epoch['tow_ns']/1e9
    info.stat = 4 if solution else 0  # RTKLIB: DGPS code solution, or no solution.
    if solution:
        info.pos = solution['position']
        info.var = [25.0, 25.0, 36.0, 0.0, 0.0, 0.0]
        info.age = 0.0
        info.dop = [0.0, solution['pdop'], 0.0, 0.0]
    grouped = {}
    for observation in epoch['observations']:
        system = observation.get('system', 'GPS')
        grouped.setdefault((system, observation['prn']), {})[observation['code']] = observation
    systems = {'GPS': (0, 1, ('1C', '2L'), (1, 17)),
               'GLONASS': (32, 4, ('1C', '2C'), (1, 14)),
               'Galileo': (59, 8, ('1C', '7Q'), (1, 28)),
               'BeiDou': (105, 32, ('2I', '7I'), (40, 27))}
    for (system, prn), signals in sorted(grouped.items()):
        offset, system_flag, signal_codes, rtklib_codes = systems[system]
        if signal_codes[0] not in signals:
            continue
        primary = signals[signal_codes[0]]
        zd = GNSS_Info_ZD()
        zd.Mea.sat = offset + prn
        zd.Mea.time = epoch['tow_ns']/1e9
        receiver_position = solution['position'] if solution else geodetic_ecef(ORIGIN)
        zd.Sat.pos = receive_frame_satellite(primary['sat_pos'], receiver_position).tolist()
        angle = 7.2921151467e-5*np.linalg.norm(np.asarray(primary['sat_pos'])-
            np.asarray(receiver_position))/C
        zd.Sat.vel = earth_rotate(primary['sat_vel'], angle).tolist()
        zd.Sat.clk_bias = primary['sat_clock_bias_s']
        zd.Sat.clk_drift = primary['sat_clock_drift_sps']
        zd.ssat.sys = system_flag
        zd.ssat.vs = 1
        zd.ssat.azel = [primary['azimuth_rad'], primary['elevation_rad']]
        lli = [0, 0, 0]
        codes = [0, 0, 0]
        for i, code in enumerate(signal_codes):
            obs = signals.get(code)
            if obs is None:
                continue
            zd.Mea.P[i] = obs['pseudorange_m']
            zd.Mea.L[i] = obs['carrier_phase_cycles']
            zd.Mea.D[i] = obs['doppler_hz']
            zd.Mea.SNR[i] = min(65535, max(0, round(obs['cn0_dbhz']*1000)))
            lli[i] = obs['lli']
            codes[i] = rtklib_codes[i]
            zd.ssat.lam[i] = C/obs['frequency_hz']
        zd.Mea.LLI = bytes(lli)
        zd.Mea.code = bytes(codes)
        info.ZD_Infos.append(zd)
    info.ns = solution['satellites'] if solution else len(info.ZD_Infos)
    return info


def solution_odom(solution, stamp):
    odom = Odometry()
    odom.header.stamp = ros_time(stamp)
    odom.header.frame_id = 'ecef'
    odom.child_frame_id = 'gnss_antenna'
    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = solution['position']
    odom.pose.pose.orientation.w = 1.0
    odom.pose.covariance[0] = 25.0
    odom.pose.covariance[7] = 25.0
    odom.pose.covariance[14] = 36.0
    return odom


def merge(args):
    report = json.loads((args.run / 'capture-report.json').read_text())
    scenario = json.loads((args.scenario or args.run / 'scenario.json').read_text())
    base_enu = scenario['receivers'].get('base_enu_m', [0.0, 0.0, 0.0])
    base_delta = vec(ecef_basis(ORIGIN), base_enu)
    base_position = [x+y for x,y in zip(geodetic_ecef(ORIGIN), base_delta)]
    origin = report['gnss_origin_sim_time_ns']
    epochs = [json.loads(line) for line in
              (args.epochs or args.run / 'signalsim/epochs.jsonl').read_text().splitlines()]
    rover = [e for e in epochs if e['receiver'] == 'rover']
    base = [e for e in epochs if e['receiver'] == 'base']
    if len(rover) != args.duration_s*5 or len(base) != len(rover):
        raise ValueError('unexpected GNSS epoch count')
    source_path = args.run / 'sensors.bag'
    target_path = args.output_bag or args.run / 'combined-signalsim.bag'
    if target_path.exists():
        raise ValueError('combined bag already exists')
    counts = Counter()
    topic_prefix = '/gnss/signalsim'
    with rosbag.Bag(str(source_path)) as source, rosbag.Bag(str(target_path), 'w') as target:
        next_epoch = 0
        for topic, msg, t in source.read_messages():
            stamp = ns(t)
            while next_epoch < len(rover) and origin+rover[next_epoch]['sim_time_ns'] <= stamp:
                e = rover[next_epoch]
                epoch_time = origin+e['sim_time_ns']
                solution = differential_position(e, base[next_epoch], base_position)
                target.write('/gnss_raw', raw_message(e, epoch_time, solution), ros_time(epoch_time))
                if solution:
                    target.write('/rtklib_odom', solution_odom(solution, epoch_time), ros_time(epoch_time))
                    counts['/rtklib_odom'] += 1
                target.write(topic_prefix + '_observations', String(data=json.dumps(e,
                    separators=(',', ':'))), ros_time(epoch_time))
                target.write(topic_prefix + '_base_observations', String(data=json.dumps(
                    base[next_epoch], separators=(',', ':'))), ros_time(epoch_time))
                counts['/gnss_raw'] += 1
                next_epoch += 1
            target.write(topic, msg, t)
            counts[topic] += 1
        if next_epoch != len(rover):
            raise ValueError('source bag ends before final GNSS epoch')
    digest = hashlib.sha256()
    with target_path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    (args.report or args.run / 'merge-report.json').write_text(json.dumps(dict(topic_counts=dict(counts),
        combined_sha256=digest.hexdigest()), indent=2)+'\n')
    print(str(target_path))


def verify(args):
    report = json.loads((args.run / 'capture-report.json').read_text())
    expected = args.duration_s*5
    records = [json.loads(line) for line in
               (args.epochs or args.run / 'signalsim/epochs.jsonl').read_text().splitlines()]
    rover_events = [e for e in records if e['receiver'] == 'rover']
    base_events = [e for e in records if e['receiver'] == 'base']
    if len(rover_events) != expected or len(base_events) != expected:
        raise ValueError('GNSS event count differs from schedule')
    for index, (r, b) in enumerate(zip(rover_events, base_events)):
        if (r['sim_time_ns'] != index*200_000_000 or
            b['sim_time_ns'] != r['sim_time_ns'] or
            (b['gps_week'], b['tow_ns']) != (r['gps_week'], r['tow_ns'])):
            raise ValueError('base/rover epoch clocks differ')
        if index:
            previous = rover_events[index-1]
            gps_delta = ((r['gps_week']-previous['gps_week'])*604800_000_000_000+
                         r['tow_ns']-previous['tow_ns'])
            if gps_delta != 200_000_000:
                raise ValueError('GPST interval differs from simulation interval')
    counts = Counter()
    max_imu_lag = max_lidar_lag = 0
    imu, lidar = [], []
    gnss = []
    valid_solutions = 0
    positions = []
    with rosbag.Bag(str(args.bag or args.run / 'combined-signalsim.bag')) as bag:
        for topic, msg, _ in bag.read_messages():
            counts[topic] += 1
            if topic == '/imu/data':
                imu.append(ns(msg.header.stamp))
            elif topic == '/velodyne_points':
                lidar.append(ns(msg.header.stamp))
            elif topic == '/gnss_raw':
                index = len(gnss)
                if index >= expected:
                    raise ValueError('extra GNSS message')
                event = rover_events[index]
                if (ns(msg.header.stamp) != report['gnss_origin_sim_time_ns']+
                    event['sim_time_ns'] or msg.gpsWeek != event['gps_week'] or
                    abs(msg.weekSec-event['tow_ns']/1e9) > 1e-6):
                    raise ValueError('ROS1 GNSS header and GPST mapping differ')
                gnss.append(ns(msg.header.stamp))
                if msg.stat == 4:
                    valid_solutions += 1
                if (msg.gpsWeek <= 0 or msg.ns < 0 or msg.ns > len(msg.ZD_Infos)
                        or (msg.stat == 4 and msg.ns < 4)):
                    raise ValueError('GNSS ROS1 message malformed')
            elif topic == '/rtklib_odom':
                positions.append((ns(msg.header.stamp), [msg.pose.pose.position.x,
                    msg.pose.pose.position.y, msg.pose.pose.position.z]))
    from bisect import bisect_left
    def nearest(stamps, target):
        index = bisect_left(stamps, target)
        nearby = stamps[max(0, index-1):index+1]
        return min(abs(target-s) for s in nearby)
    if not imu or not lidar:
        raise ValueError('IMU or LiDAR missing')
    imu_lags = [nearest(imu, t) for t in gnss]
    lidar_lags = [nearest(lidar, t) for t in gnss]
    max_imu_lag = max(imu_lags)
    max_lidar_lag = max(lidar_lags)
    imu_outside = sum(lag > 20_000_000 for lag in imu_lags)
    import sys
    sys.path.insert(0, str(HERE.parents[2]))
    from apps.fastglio.trajectory import antenna_state  # noqa: E402
    document = json.loads((args.run / 'trajectory.json').read_text())
    stamps = [sample['sim_time_ns'] for sample in document['samples']]
    position_errors = [math.dist(position, antenna_state(document, stamp, stamps)[0])
                       for stamp, position in positions]
    ok = (len(gnss) == expected and len(set(gnss)) == expected and
          all(b-a == 200_000_000 for a,b in zip(gnss,gnss[1:])) and
          imu_outside <= expected//100 and max_imu_lag <= 50_000_000 and
          max_lidar_lag <= 100_000_000 and
          valid_solutions >= max(5, expected//20) and
          counts['/rtklib_odom'] == valid_solutions and
          (not position_errors or max(position_errors) <= 30.0) and
          counts['/ground_truth/state'] > 0 and counts['/clock'] > 0)
    result = dict(pass_quality_gate=ok, duration_s=args.duration_s,
        origin_lla_deg_m=ORIGIN, gnss_epochs=len(gnss), expected_epochs=expected,
        max_imu_offset_ms=max_imu_lag/1e6, max_lidar_offset_ms=max_lidar_lag/1e6,
        imu_epochs_outside_20ms=imu_outside,
        imu_outside_fraction=imu_outside/expected,
        degraded=imu_outside > 0,
        dgnss_code_solution_epochs=valid_solutions,
        dgnss_code_solution_max_truth_error_m=max(position_errors, default=None),
        topic_counts=dict(counts), capture=report)
    (args.report or args.run / 'verification.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    if not ok:
        raise ValueError('quality gate failed; inspect verification.json')


def main():
    global ORIGIN
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('capture', 'trajectory', 'merge', 'verify'):
        cmd = sub.add_parser(name)
        cmd.add_argument('--run', type=Path, required=True)
        cmd.add_argument('--duration-s', type=int, default=300)
        if name == 'capture':
            cmd.add_argument('--interactive', action='store_true')
            cmd.add_argument('--mapping-only', action='store_true')
            cmd.add_argument('--map-script', type=Path)
            cmd.add_argument('--web-console-script', type=Path)
            cmd.add_argument('--web-console-port', type=int, default=8766)
            cmd.add_argument('--gui', action='store_true')
            cmd.add_argument('--launch-file', type=Path,
                             help='ROS launch file for an alternate vehicle simulator')
            cmd.add_argument('--launch-arg', action='append', default=[],
                             help='roslaunch name:=value argument; repeat as needed')
            cmd.add_argument('--controller-script', type=Path,
                             help='flight script started after all sensor topics are ready')
            cmd.add_argument('--injector-script', type=Path,
                             help='online GNSS source started before the flight controller')
            cmd.add_argument('--gazebo-viz-script', type=Path,
                             help='Gazebo overlay for live GNSS spoofing experiment')
            cmd.add_argument('--fastglio-launch', type=Path,
                             help='Start live FAST_GLIO alongside Gazebo')
            cmd.add_argument('--fastglio-config', type=Path)
            cmd.add_argument('--fastglio-point-filter-num', type=int, default=2)
            cmd.add_argument('--fastglio-rviz-config', type=Path)
            cmd.add_argument('--tile-windows-script', type=Path)
            cmd.add_argument('--flight-mode', default='patrol_s', choices=[
                'patrol_s', 'patrol_8', 'patrol_boustrophedon',
                'patrol_sprint', 'patrol_spiral'])
        if name == 'trajectory':
            cmd.add_argument('--antenna-offset', type=float, nargs=3, default=LEVER,
                             metavar=('X', 'Y', 'Z'),
                             help='GPS antenna lever arm in the vehicle base frame, metres')
        if name == 'merge':
            cmd.add_argument('--epochs', type=Path)
            cmd.add_argument('--scenario', type=Path)
            cmd.add_argument('--output-bag', type=Path)
            cmd.add_argument('--report', type=Path)
        if name == 'verify':
            cmd.add_argument('--epochs', type=Path)
            cmd.add_argument('--bag', type=Path)
            cmd.add_argument('--report', type=Path)
    args = parser.parse_args()
    archived_profile = args.run/'scene-profile.json'
    if archived_profile.is_file():
        ORIGIN = load_scene_profile(archived_profile)['origin_lla_deg_m']
    if args.duration_s < 2 and not (args.duration_s == 0 and
            (args.command == 'trajectory' or args.command == 'capture' and args.interactive)):
        parser.error('duration must be at least 2 seconds')
    try:
        globals()[args.command](args)
    except KeyboardInterrupt:
        if args.command == 'capture' and args.interactive:
            print('已取消手动采集启动。', flush=True)
        else:
            raise


if __name__ == '__main__':
    main()
