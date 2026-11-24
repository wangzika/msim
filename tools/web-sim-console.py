#!/usr/bin/env python3
"""Local browser console for headless Gazebo/PX4/FAST_GLIO captures."""

import argparse
from collections import deque
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import shlex
from pathlib import Path
import struct
import sys
import threading
import time
from urllib.parse import parse_qs, urlsplit, unquote
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parent))
from web_cloud import MAX_MAP_POINTS, MAX_SCAN_POINTS, pcd_preview
from web_scene import scene_resources


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'adapters/ros1_noetic/msim_sensors'))
from sensor_profiles import GPS, DEFAULT_GPS
STATIC = ROOT / 'web/sim-console'
MAX_SAMPLES = 3000
MOVEMENT_KEYS = frozenset('wasdrfqe')
SCENES = (
    dict(key='simple', name='城市走廊', suggested_takeoff_height_m=8,description='原始走廊，便于快速检查与对照'),
    dict(key='complex_urban', name='复杂街区',suggested_takeoff_height_m=8, description='错落建筑、立柱、斜墙与高架桥'),
    dict(key='industrial_yard', name='工业场地',suggested_takeoff_height_m=8, description='货架、托盘、货箱与多高度结构'),
    dict(key='ksql_airport', name='KSQL 机场',suggested_takeoff_height_m=12, description='约 2.87 × 2.78 km，保留地形与建筑网格'),
)


def scene_metadata(run, settings):
    profile = read_json(run/'scene-profile.json',{})
    key = settings.get('scene_key')
    if key != 'custom' and key not in {s['key'] for s in SCENES}:
        key = next((s['key'] for s in SCENES if s['key'] in str(settings.get('world_source',''))), None)
    if key is None and profile.get('name') == 'ksql_airport':key='ksql_airport'
    selected = next((s for s in SCENES if s['key']==key),dict(key=key or 'custom',name=profile.get('name','自定义场景')))
    sensors=read_json(run/'sensor_profile.json',{})
    return dict(**selected, choices=SCENES, profile=profile,command_path=str(ROOT/'tools/px4-uav-capture.sh'),
                sensor_models=sensors.get('models',{}),lidar_timing=sensors.get('lidar',{}).get('timing','rolling'),
                lidar_model=sensors.get('models',{}).get('lidar','vlp16'),
                has_meshes=bool(ET.parse(run/'world.sdf').findall('.//mesh')) if (run/'world.sdf').is_file() else False)


def numbers(text, count):
    try:
        values = [float(value) for value in text.split()]
    except (ValueError, AttributeError):
        return None
    return values if len(values) >= count and all(math.isfinite(v) for v in values) else None


def launch_script(run, key, mode):
    selected=next((scene for scene in SCENES if scene['key']==key),None)
    if selected is None or mode not in ('teleop','route'):raise ValueError('Invalid scene or capture mode')
    settings=read_json(run/'capture-settings.json',{})
    info=scene_metadata(run,settings);models=info['sensor_models']
    arguments=[str(ROOT/'tools/px4-uav-capture.sh'),'--web']
    if mode=='teleop':
        height=settings.get('teleop_takeoff_height_m') if key==info['key'] else None
        arguments+=['--teleop','--teleop-height-m',str(height or selected['suggested_takeoff_height_m'])]
    else:arguments+=['--duration-s','300','--flight-pattern','route','--route-clock','sim','--skip-signalsim']
    arguments+=['--scene',key]
    for option,field,allowed,default in (
        ('--lidar-model','lidar',('vlp16','hdl32','os1-64','mid360'),'vlp16'),
        ('--imu-model','imu',('mems-baseline','low-noise'),'mems-baseline'),
        ('--gps-model','gps',tuple(GPS),DEFAULT_GPS)):
        value=models.get(field,default);arguments+=[option,value if value in allowed else default]
    timing=info['lidar_timing'];arguments+=['--lidar-timing',timing if timing in ('rolling','snapshot') else 'rolling']
    stride=settings.get('point_filter_num',2)
    arguments+=['--point-filter-num',str(stride if type(stride) is int and 0<stride<=100 else 2)]
    return ('#!/usr/bin/env bash\nset -euo pipefail\nexec '+shlex.join(arguments)+'\n').encode()


@lru_cache(maxsize=8)
def scene_geometry(path):
    """Extract lightweight primitives from top-level SDF models (ENU metres)."""
    if not path.is_file():
        return []
    world = ET.parse(path).getroot().find('world')
    if world is None:
        return []
    shapes = []
    for model in world.findall('model'):
        model_pose = numbers(model.findtext('pose', '0 0 0 0 0 0'), 6)
        if model_pose is None:
            continue
        for link in model.findall('link'):
            link_pose = numbers(link.findtext('pose', '0 0 0 0 0 0'), 6)
            if link_pose is None:
                continue
            for collision in link.findall('collision'):
                pose = numbers(collision.findtext('pose', '0 0 0 0 0 0'), 6)
                if pose is None:
                    continue
                geometry = collision.find('geometry')
                if geometry is None:
                    continue
                box = geometry.find('box')
                cylinder = geometry.find('cylinder')
                size = numbers(box.findtext('size'), 3) if box is not None else None
                if size is None and cylinder is not None:
                    radius = numbers(cylinder.findtext('radius'), 1)
                    length = numbers(cylinder.findtext('length'), 1)
                    if radius and length:
                        size = [2 * radius[0], 2 * radius[0], length[0]]
                if size is None or min(size) <= 0:
                    continue
                # SDF poses are composed at the model/link/collision levels.
                # Existing box scenes have no link/collision roll or pitch.
                yaw = model_pose[5] + link_pose[5] + pose[5]
                c, s = math.cos(model_pose[5]), math.sin(model_pose[5])
                lx = link_pose[0] + pose[0]
                ly = link_pose[1] + pose[1]
                shapes.append(dict(name=model.get('name', ''),
                    center=[model_pose[0] + c * lx - s * ly,
                            model_pose[1] + s * lx + c * ly,
                            model_pose[2] + link_pose[2] + pose[2]],
                    size=size, yaw=yaw))
                if len(shapes) >= 600:
                    return shapes
    return shapes


def read_samples(path):
    if not path.is_file():
        return []
    rows = deque(maxlen=MAX_SAMPLES)
    with path.open() as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue  # The recorder may be writing the last line.
            if 'sim_time_ns' in row and 'east_m' in row:
                rows.append({key: row.get(key) for key in (
                    'sim_time_ns', 'east_m', 'north_m', 'up_m',
                    'gps_east_m', 'gps_north_m', 'gps_up_m',
                    'detector_status', 'attack_active', 'sky_blocked_fraction',
                    'environment_noise_scale', 'lidar_sync_status')})
    return list(rows)


def read_json(path, fallback):
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return fallback


def validate_command(value):
    if not isinstance(value, dict) or value.get('op') not in (
            'keys', 'takeoff', 'hold', 'land', 'finish', 'speed'):
        raise ValueError('Unknown teleoperation command')
    op = value['op']
    if op == 'keys':
        keys = value.get('keys')
        if not isinstance(keys, list) or len(keys) > 8 or any(
                not isinstance(key, str) or key not in MOVEMENT_KEYS for key in keys):
            raise ValueError('Invalid movement keys')
        return dict(op=op, keys=sorted(set(keys)))
    if op in ('speed', 'takeoff'):
        key = 'value' if op == 'speed' else 'height_m'
        value_number = value.get(key)
        if isinstance(value_number, bool) or not isinstance(value_number, (int, float)) or \
                not math.isfinite(value_number) or not (
                    .2 <= value_number <= 8 if op == 'speed' else .5 <= value_number <= 30):
            raise ValueError('Invalid speed or takeoff height')
        return {'op': op, key: value_number}
    return {'op': op}


class LiveRos:
    def __init__(self, run=None):
        import numpy as np
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String

        self.lock = threading.Lock()
        sys.path.insert(0,str(ROOT/'adapters/ros1_noetic/fast_glio_uav'))
        from lio_map_recorder import VoxelMap, cloud_points
        self.run = run
        self.cache_at = 0.
        self.tracks = {'lio': deque(maxlen=10000), 'loose': deque(maxlen=10000)}
        self.pose = {};self.frames = {};self.track_at = {}
        self.packets = {};self.revision = 0
        self.grid = VoxelMap(.4,400000)
        self.cloud_at = 0.;self.last_map = 0.;self.last_external_map = -math.inf
        rospy.init_node('msim_web_console', anonymous=True, disable_signals=True)
        self.publisher = rospy.Publisher('/msim/teleop/command', String, queue_size=5, tcp_nodelay=True)

        def odom_callback(topic):
            def on_message(message):
                p = message.pose.pose.position
                q = message.pose.pose.orientation
                if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):return
                now=time.monotonic()
                with self.lock:
                    self.pose[topic]=dict(position=[p.x,p.y,p.z],orientation=[q.x,q.y,q.z,q.w])
                    self.frames[topic]=message.header.frame_id
                    if now-self.track_at.get(topic,0)>=.15:
                        self.tracks[topic].append([p.x, p.y, p.z]);self.track_at[topic]=now
            return on_message

        def packet(points, total, frame, source):
            self.revision+=1
            return dict(data=np.asarray(points[:,:3],dtype='<f4').tobytes(), count=len(points),
                        total=total, frame=frame, revision=str(self.revision), source=source, at=time.monotonic())

        def truth_pose(message):
            p=message.pose.pose.position;q=message.pose.pose.orientation
            if not all(math.isfinite(value) for value in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):return
            with self.lock:
                self.pose['truth']=dict(position=[p.x,p.y,p.z],orientation=[q.x,q.y,q.z,q.w])
                self.frames['truth']=message.header.frame_id

        def on_cloud(message, accumulated=False):
            now = time.monotonic()
            if not accumulated and now-self.cloud_at<.25:return
            points=cloud_points(message);points=points[np.isfinite(points).all(1)]
            if message.header.frame_id != 'camera_init':return
            with self.lock:
                if accumulated:
                    sampled=points[np.linspace(0,len(points)-1,min(len(points),MAX_MAP_POINTS),dtype=int)] if len(points) else points
                    self.packets['map']=packet(sampled,len(points),message.header.frame_id,'/msim/lio_map')
                    self.last_external_map=now
                else:
                    sampled=points[np.linspace(0,len(points)-1,min(len(points),MAX_SCAN_POINTS),dtype=int)] if len(points) else points
                    self.packets['current']=packet(sampled,len(points),message.header.frame_id,'/cloud_registered')
                    self.cloud_at=now
                    if now-self.last_external_map>2:
                        self.grid.add(points)
                        if now-self.last_map>=1:
                            values=self.grid.array()
                            sampled=values[np.linspace(0,len(values)-1,min(len(values),MAX_MAP_POINTS),dtype=int)] if len(values) else values
                            self.packets['map']=packet(sampled,len(values),message.header.frame_id,'registered_scan_accumulation')
                            self.last_map=now

        self.subscribers = [
            rospy.Subscriber('/ground_truth/state',Odometry,truth_pose,queue_size=1,tcp_nodelay=True),
            rospy.Subscriber('/Odometry', Odometry, odom_callback('lio'), queue_size=2),
            rospy.Subscriber('/loose_odom', Odometry, odom_callback('loose'), queue_size=2),
            rospy.Subscriber('/cloud_registered', PointCloud2, on_cloud,
                             queue_size=1, buff_size=16 * 1024 * 1024),
            rospy.Subscriber('/msim/lio_map',PointCloud2,lambda m:on_cloud(m,True),
                             queue_size=1,buff_size=32*1024*1024),
        ]

    def snapshot(self, force_persist=False):
        with self.lock:
            result = dict(lio=list(self.tracks['lio']),loose=list(self.tracks['loose']),
                          pose=self.pose.copy(),frames=self.frames.copy(),cloud_frame='camera_init',
                          cloud_layers={key:{k:v for k,v in packet.items() if k!='data'} for key,packet in self.packets.items()})
            if self.run and (force_persist or time.monotonic()-self.cache_at>2):
                temporary=self.run/'web-lio-state.json.tmp';temporary.write_text(json.dumps(result)+'\n');temporary.replace(self.run/'web-lio-state.json')
                self.cache_at=time.monotonic()
        return result

    def cloud_packet(self, layer):
        with self.lock:
            value=self.packets.get(layer)
            if value is None:return None
            result=dict(value);result['age_s']=max(0,time.monotonic()-result.pop('at'))
            return result

    def pose_snapshot(self):
        with self.lock:return dict(pose=self.pose.copy(),frames=self.frames.copy())

    def command(self, value):
        from std_msgs.msg import String
        self.publisher.publish(String(data=json.dumps(value, separators=(',', ':'))))


def make_handler(run, ros=None):
    class Handler(BaseHTTPRequestHandler):
        def local_request(self):
            return self.headers.get('Host', '').split(':')[0] in ('127.0.0.1', 'localhost')

        def respond(self, payload, mime='application/json', code=200):
            if not isinstance(payload, bytes):
                payload = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:self.wfile.write(payload)
            except (BrokenPipeError,ConnectionResetError):pass

        def send_cloud(self, layer, revision):
            packet=ros.cloud_packet(layer) if ros else None
            if packet is None and layer=='map' and (run/'map.pcd').is_file():
                try:packet=pcd_preview(run/'map.pcd')
                except (ValueError,KeyError,OSError,UnicodeError) as error:
                    self.respond(dict(error=str(error)),code=422);return
            if packet is None:
                self.respond(b'',code=204);return
            unchanged=revision==packet['revision']
            self.send_response(204 if unchanged else 200)
            self.send_header('Content-Type','application/octet-stream')
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Cloud-Revision',str(packet['revision']))
            self.send_header('X-Cloud-Frame',str(packet['frame']))
            self.send_header('X-Point-Count',str(packet['count']))
            self.send_header('X-Point-Total',str(packet['total']))
            self.send_header('X-Cloud-Source',packet['source'])
            self.send_header('X-Cloud-Age',str(packet.get('age_s') or 0))
            self.send_header('Content-Length',str(0 if unchanged else len(packet['data'])))
            self.end_headers()
            if not unchanged:
                try:self.wfile.write(packet['data'])
                except (BrokenPipeError,ConnectionResetError):pass

        def do_GET(self):
            if not self.local_request():
                self.send_error(403)
                return
            url=urlsplit(self.path);path=unquote(url.path)
            if path=='/api/poses':
                snapshot=ros.pose_snapshot() if ros else read_json(run/'web-lio-state.json',{})
                self.respond(dict(pose=snapshot.get('pose',{}),frames=snapshot.get('frames',{})))
                return
            if path=='/api/scene-models':
                try:self.respond(scene_resources(run/'world.sdf').manifest())
                except (OSError,ValueError,ET.ParseError) as error:self.respond(dict(error=str(error)),code=422)
                return
            if path.startswith('/api/scene-assets/'):
                token=path.removeprefix('/api/scene-assets/')
                if '/' in token or '..' in token:self.send_error(404);return
                resources=scene_resources(run/'world.sdf');asset=resources.assets.get(token)
                if asset is None:self.send_error(404);return
                mime={'.dae':'model/vnd.collada+xml','.stl':'application/octet-stream','.png':'image/png','.jpg':'image/jpeg','.jpeg':'image/jpeg','.webp':'image/webp','.bmp':'image/bmp','.tga':'application/octet-stream'}[asset.suffix.lower()]
                self.respond(resources.materialized.get(token) or asset.read_bytes(),mime);return
            if path=='/api/launch.sh':
                query=parse_qs(url.query);key=query.get('scene',[''])[0];mode=query.get('mode',[''])[0]
                try:payload=launch_script(run,key,mode)
                except ValueError:
                    self.respond(dict(error='Invalid scene or mode'),code=400);return
                self.send_response(200);self.send_header('Content-Type','text/x-shellscript; charset=utf-8')
                self.send_header('Content-Disposition',f'attachment; filename="msim-{key}-{mode}.sh"')
                self.send_header('Content-Length',str(len(payload)));self.end_headers()
                try:self.wfile.write(payload)
                except (BrokenPipeError,ConnectionResetError):pass
                return
            if path=='/api/cloud':
                query=parse_qs(url.query);layer=query.get('layer',['map'])[0]
                if layer not in ('map','current'):
                    self.respond(dict(error='Unknown point-cloud layer'),code=400);return
                self.send_cloud(layer,query.get('revision',[''])[0]);return
            if path=='/api/map.pcd':
                map_path=run/'map.pcd'
                if not map_path.is_file():self.send_error(404);return
                try:
                    with map_path.open('rb') as stream:
                        self.send_response(200);self.send_header('Content-Type','application/octet-stream')
                        self.send_header('Content-Disposition','attachment; filename="map.pcd"')
                        self.send_header('Content-Length',str(os.fstat(stream.fileno()).st_size));self.end_headers()
                        while chunk:=stream.read(1024*1024):self.wfile.write(chunk)
                except (BrokenPipeError,ConnectionResetError):pass
                return
            if path.startswith('/exports/'):
                target=(run/'web-exports'/path.removeprefix('/exports/')).resolve()
                if target.parent != (run/'web-exports').resolve() or not target.is_file() or target.suffix != '.png':
                    self.send_error(404);return
                self.respond(target.read_bytes(),'image/png');return
            if path in ('/','/index.html','/app.js','/style.css','/slam-viewer.js','/flight-input.js','/scene-command.js') or path.startswith('/vendor/'):
                filename = 'index.html' if path == '/' else path[1:]
                target=(STATIC/filename).resolve()
                if not target.is_relative_to(STATIC.resolve()) or not target.is_file():
                    self.send_error(404);return
                mime = ('text/html; charset=utf-8' if filename.endswith('.html') else
                        'text/javascript; charset=utf-8' if filename.endswith('.js') else
                        'text/css; charset=utf-8')
                self.respond(target.read_bytes(), mime)
            elif path == '/api/state':
                rows = read_samples(run / 'injected-gps.jsonl')
                route = read_json(run / 'route_plan.json', [])
                status = read_json(run / 'teleop-status.json', {})
                settings = read_json(run / 'capture-settings.json', {})
                failure=read_json(run/'capture-failure.json',{})
                complete=not ros or any((run/name).is_file() for name in ('capture-report.json','capture-session.json','capture-failure.json'))
                self.respond(dict(run=run.name, complete=complete,failure=failure,streaming=bool(ros and not complete),
                                  settings=settings, samples=rows, route=route,
                                  control_available=bool(ros and settings.get('interactive') and not complete),
                                  teleop=status, scene=scene_geometry(run / 'world.sdf'),
                                  scene_info=scene_metadata(run,settings),
                                  map_saved=(run/'map.pcd').is_file(),
                                  mapping=read_json(run/'mapping-report.json',{}),
                                  live=ros.snapshot() if ros else read_json(run/'web-lio-state.json',{})))
            else:
                self.send_error(404)

        def do_POST(self):
            if not self.local_request():
                self.send_error(403)
                return
            origin = self.headers.get('Origin')
            host = self.headers.get('Host', '')
            if origin and origin not in (f'http://{host}', f'https://{host}'):
                self.respond({'error': 'Origin mismatch'}, code=403)
                return
            if self.path == '/api/export/png':
                try:
                    length=int(self.headers.get('Content-Length','0'))
                    if not 24 <= length <= 12*1024*1024:raise ValueError('Invalid PNG size')
                    payload=self.rfile.read(length)
                    if len(payload)!=length or payload[:16]!=b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR':raise ValueError('Invalid PNG header')
                    width,height=struct.unpack('>II',payload[16:24])
                    if not 1 <= width <= 16384 or not 1 <= height <= 16384:raise ValueError('Invalid PNG dimensions')
                    directory=run/'web-exports';directory.mkdir(exist_ok=True)
                    filename=f'slam-{time.time_ns()}.png'
                    (directory/filename).write_bytes(payload)
                    self.respond(dict(url=f'/exports/{filename}',file=str(directory/filename)))
                except (OSError,ValueError) as error:self.respond(dict(error=str(error)),code=400)
                return
            if self.path != '/api/teleop/command':
                self.send_error(404)
                return
            settings = read_json(run / 'capture-settings.json', {})
            if ros is None or not settings.get('interactive') or any((run/name).is_file() for name in ('capture-report.json','capture-session.json','capture-failure.json')):
                self.respond({'error': 'Web control is available only in live teleop mode'}, code=409)
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 512:
                    raise ValueError('Command too large')
                command = validate_command(json.loads(self.rfile.read(length)))
            except (ValueError, json.JSONDecodeError):
                self.respond({'error': 'Invalid command'}, code=400)
                return
            ros.command(command)
            self.respond({'ok': True})

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--ros', action='store_true', help='stream LIO and enable live teleop')
    args = parser.parse_args()
    if not args.run.is_dir():
        parser.error('run directory does not exist')
    bridge = LiveRos(args.run) if args.ros else None
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(args.run, bridge))
    print(f'网页仿真控制台：http://127.0.0.1:{args.port}/', flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        if bridge:bridge.snapshot(force_persist=True)
        server.server_close()


if __name__ == '__main__':
    main()
