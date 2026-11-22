#!/usr/bin/env python3
"""Local browser console for headless Gazebo/PX4/FAST_GLIO captures."""

import argparse
from collections import deque
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
STATIC = ROOT / 'web/sim-console'
MAX_SAMPLES = 700
MAX_CLOUD_POINTS = 3000
MOVEMENT_KEYS = frozenset('wasdrfqe')


def numbers(text, count):
    try:
        values = [float(value) for value in text.split()]
    except (ValueError, AttributeError):
        return None
    return values if len(values) >= count and all(math.isfinite(v) for v in values) else None


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
    def __init__(self):
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        from std_msgs.msg import String

        self.lock = threading.Lock()
        self.tracks = {'lio': deque(maxlen=600), 'loose': deque(maxlen=600)}
        self.cloud = []
        self.cloud_frame = ''
        self.cloud_at = 0.
        rospy.init_node('msim_web_console', anonymous=True, disable_signals=True)
        self.publisher = rospy.Publisher('/msim/teleop/command', String, queue_size=10)

        def odom_callback(topic):
            def on_message(message):
                p = message.pose.pose.position
                with self.lock:
                    self.tracks[topic].append([p.x, p.y, p.z])
            return on_message

        def on_cloud(message):
            now = time.monotonic()
            if now - self.cloud_at < 1.0:
                return
            stride = max(1, message.width * message.height // MAX_CLOUD_POINTS)
            points = []
            for index, point in enumerate(point_cloud2.read_points(
                    message, field_names=('x', 'y', 'z'), skip_nans=True)):
                if index % stride == 0 and all(math.isfinite(v) for v in point):
                    points.append([round(float(v), 2) for v in point])
                    if len(points) >= MAX_CLOUD_POINTS:
                        break
            with self.lock:
                self.cloud = points
                self.cloud_frame = message.header.frame_id
                self.cloud_at = now

        self.subscribers = [
            rospy.Subscriber('/Odometry', Odometry, odom_callback('lio'), queue_size=2),
            rospy.Subscriber('/loose_odom', Odometry, odom_callback('loose'), queue_size=2),
            rospy.Subscriber('/cloud_registered', PointCloud2, on_cloud,
                             queue_size=1, buff_size=16 * 1024 * 1024),
        ]

    def snapshot(self):
        with self.lock:
            return dict(lio=list(self.tracks['lio']), loose=list(self.tracks['loose']),
                        cloud=self.cloud, cloud_frame=self.cloud_frame)

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
            self.wfile.write(payload)

        def do_GET(self):
            if not self.local_request():
                self.send_error(403)
                return
            if self.path in ('/', '/index.html', '/app.js', '/style.css'):
                filename = 'index.html' if self.path == '/' else self.path[1:]
                mime = ('text/html; charset=utf-8' if filename.endswith('.html') else
                        'text/javascript; charset=utf-8' if filename.endswith('.js') else
                        'text/css; charset=utf-8')
                self.respond((STATIC / filename).read_bytes(), mime)
            elif self.path == '/api/state':
                rows = read_samples(run / 'injected-gps.jsonl')
                route = read_json(run / 'route_plan.json', [])
                status = read_json(run / 'teleop-status.json', {})
                settings = read_json(run / 'capture-settings.json', {})
                self.respond(dict(run=run.name, complete=(run / 'capture-report.json').is_file(),
                                  settings=settings, samples=rows, route=route,
                                  teleop=status, scene=scene_geometry(run / 'world.sdf'),
                                  live=ros.snapshot() if ros else {}))
            else:
                self.send_error(404)

        def do_POST(self):
            if not self.local_request():
                self.send_error(403)
                return
            if self.path != '/api/teleop/command':
                self.send_error(404)
                return
            origin = self.headers.get('Origin')
            host = self.headers.get('Host', '')
            if origin and origin not in (f'http://{host}', f'https://{host}'):
                self.respond({'error': 'Origin mismatch'}, code=403)
                return
            settings = read_json(run / 'capture-settings.json', {})
            if ros is None or not settings.get('interactive'):
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
    bridge = LiveRos() if args.ros else None
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(args.run, bridge))
    print(f'网页仿真控制台：http://127.0.0.1:{args.port}/', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
