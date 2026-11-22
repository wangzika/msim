#!/usr/bin/env python3
"""Show live truth, injected GNSS and detector estimate inside Gazebo Classic."""

import json
import math
import os
from pathlib import Path

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SetModelState, SpawnModel
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import String


COLORS = {
    'truth': ('1 1 1', 0.11),
    'gnss': ('0.15 0.55 1', 0.12),
    'estimate': ('0.1 1 0.4', 0.10),
    'spoof_reference': ('1 0.1 1', 0.11),
    'alarm': ('1 0.08 0.08', 0.16),
}
ROUTE_COLOR = '1 0.78 0.08'
MAX_TRAIL_POINTS = 40
TRAIL_PERIOD_NS = 1_000_000_000
ROUTE_PLAN = Path(os.environ.get('MSIM_ROUTE_PLAN',
                                 str(Path(__file__).with_name('route_plan.json'))))


def marker_sdf(name, color, radius):
    """A visual-only sphere: no collision and no gravity influence on flight."""
    return f'''<sdf version="1.6"><model name="{name}">
      <static>false</static><link name="visual_link"><gravity>false</gravity>
      <visual name="colored_sphere"><cast_shadows>false</cast_shadows>
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse>
          <emissive>{color} 1</emissive></material>
      </visual></link></model></sdf>'''


def pose(east, north, up):
    result = Pose()
    result.position.x, result.position.y, result.position.z = east, north, up
    result.orientation.w = 1.0
    return result


def dashed_visuals(prefix, start, end, color, spacing=0.65):
    """Make short emissive boxes along a 3D segment, with visible gaps."""
    dx, dy, dz = (end[i] - start[i] for i in range(3))
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 0.08:
        return []
    count = min(48, max(1, math.ceil(length / spacing)))
    dash_length = length * 0.58 / count
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    visuals = []
    for index in range(count):
        fraction = (index + 0.5) / count
        x, y, z = (start[i] + fraction * (end[i] - start[i]) for i in range(3))
        visuals.append(f'''<visual name="{prefix}_{index}"><cast_shadows>false</cast_shadows>
          <pose>{x} {y} {z} 0 {pitch} {yaw}</pose>
          <geometry><box><size>{dash_length} 0.045 0.045</size></box></geometry>
          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse>
          <emissive>{color} 1</emissive></material></visual>''')
    return visuals


def trail_sdf(name, color, start, end):
    visuals = dashed_visuals('dash', start, end, color)
    return (f'<sdf version="1.6"><model name="{name}"><static>true</static>'
            '<link name="trail_link">' + ''.join(visuals) + '</link></model></sdf>')


def route_sdf(points, east0, north0, height):
    """One visual-only Gazebo model containing the planned route and waypoints."""
    visuals = []
    color = ROUTE_COLOR
    for index, (start, end) in enumerate(zip(points, points[1:])):
        visuals.extend(dashed_visuals(
            f'segment_{index}',
            (east0 + start['east_m'], north0 + start['north_m'], height),
            (east0 + end['east_m'], north0 + end['north_m'], height),
            color))
    for index, point in enumerate(points):
        x, y = east0 + point['east_m'], north0 + point['north_m']
        visuals.append(f'''<visual name="waypoint_{index}"><pose>{x} {y} {height} 0 0 0</pose>
          <geometry><sphere><radius>0.12</radius></sphere></geometry>
          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse>
          <emissive>{color} 1</emissive></material></visual>''')
    return '<sdf version="1.6"><model name="msim_planned_route"><static>true</static>' \
           '<link name="route_link">' + ''.join(visuals) + '</link></model></sdf>'


class GazeboSpoofViz:
    def __init__(self, spawn, move, delete):
        self.spawn = spawn
        self.move = move
        self.delete = delete
        self.models = set()
        self.latest_truth = None
        self.latest_detection = None
        self.alarm_seen = False
        self.last_trail_ns = None
        self.trail_sequence = 0
        self.trail_slots = {}
        self.last_trail_positions = {}
        self.spoof_trail_seen = False
        self.route_origin_east = None
        self.route_points = (json.loads(ROUTE_PLAN.read_text())
                             if os.environ.get('MSIM_PX4_FLIGHT_PATTERN') == 'route' else None)

    def on_truth(self, msg):
        p = msg.pose.pose.position
        self.latest_truth = (msg.header.stamp.to_nsec(), p.x, p.y, p.z)

    def on_detection(self, msg):
        try:
            row = json.loads(msg.data)
            if all(key in row for key in ('sim_time_ns', 'gps_east_m',
                                          'gps_north_m', 'estimate_east_m',
                                          'estimate_north_m', 'detector_status')):
                self.latest_detection = row
        except (ValueError, TypeError):
            rospy.logwarn_throttle(5, 'invalid /gnss/spoof_detection JSON')

    def _put(self, name, kind, position):
        where = pose(*position)
        if name not in self.models:
            color, radius = COLORS[kind]
            response = self.spawn(name, marker_sdf(name, color, radius),
                                  '', where, 'world')
            if not response.success:
                raise RuntimeError(f'Gazebo could not spawn {name}: {response.status_message}')
            self.models.add(name)
        else:
            state = ModelState()
            state.model_name = name
            state.pose = where
            state.reference_frame = 'world'
            response = self.move(state)
            if not response.success:
                raise RuntimeError(f'Gazebo could not move {name}: {response.status_message}')

    def _put_trail_segment(self, name, kind, start, end):
        color = COLORS[kind][0]
        xml = trail_sdf(name, color, start, end)
        response = self.spawn(name, xml, '', pose(0, 0, 0), 'world')
        if not response.success:
            raise RuntimeError(f'Gazebo could not spawn {name}: {response.status_message}')
        self.models.add(name)
        if kind == 'spoof_reference' and not self.spoof_trail_seen:
            self.spoof_trail_seen = True
            print('Gazebo purple spoof offset trail created', flush=True)

    def update(self):
        truth, row = self.latest_truth, self.latest_detection
        if truth is None or row is None:
            return False
        if abs(truth[0] - row['sim_time_ns']) > 500_000_000:
            return False
        # Detector state is 2D. Display its horizontal estimate at the
        # aircraft's real height without introducing a visual-only offset.
        display_z = truth[3]
        if self.route_points is not None and 'msim_planned_route' not in self.models:
            self.route_origin_east = truth[1]
            from scene_profile import from_environment
            route_height = truth[3] + from_environment()['flight_height_m']
            route = route_sdf(self.route_points, truth[1], truth[2], route_height)
            response = self.spawn('msim_planned_route', route, '', pose(0, 0, 0), 'world')
            if not response.success:
                raise RuntimeError(f'Gazebo could not spawn planned route: {response.status_message}')
            self.models.add('msim_planned_route')
        positions = {
            'truth': (truth[1], truth[2], display_z),
            'gnss': (row['gps_east_m'], row['gps_north_m'], row.get('gps_up_m', display_z)),
            'estimate': (row['estimate_east_m'], row['estimate_north_m'], display_z),
        }
        if self.route_points is not None and row.get('attack_active'):
            # A visual reference for the injected displacement. The actual
            # GPS fed to PX4 is the blue marker, which can remain near the route
            # as the aircraft physically moves in the opposite direction.
            positions['spoof_reference'] = (
                self.route_origin_east + row.get('east_bias_m', 0) + row.get('attack_east_m', 0),
                truth[2] + row.get('north_bias_m', 0) + row.get('attack_north_m', 0),
                display_z)
        for kind, position in positions.items():
            self._put(f'msim_{kind}_live', kind, position)
        stamp = row['sim_time_ns']
        if row['detector_status'] == 'alarm' and not self.alarm_seen:
            self.alarm_seen = True
            print(f'Gazebo red alarm marker at simulation {stamp / 1e9:.1f} s',
                  flush=True)
        if self.alarm_seen:
            self._put('msim_alarm_red', 'alarm',
                      (truth[1], truth[2], display_z + 0.4))
        if self.last_trail_ns is None or stamp - self.last_trail_ns >= TRAIL_PERIOD_NS:
            # Keep a bounded history, but never reuse model/visual names.
            # gzclient releases Ogre materials asynchronously after deletion;
            # reusing a slot's old name can throw ItemIdentityException and crash.
            slot = self.trail_sequence % MAX_TRAIL_POINTS
            for name in self.trail_slots.pop(slot, []):
                if name in self.models:
                    response = self.delete(name)
                    if not response.success:
                        raise RuntimeError(f'Gazebo could not delete {name}: {response.status_message}')
                    self.models.remove(name)
            created = []
            for kind, position in positions.items():
                previous = self.last_trail_positions.get(kind)
                self.last_trail_positions[kind] = position
                if previous is None or math.dist(previous, position) < 0.08:
                    continue
                name = f'msim_{kind}_trail_{self.trail_sequence:08d}'
                self._put_trail_segment(name, kind, previous, position)
                created.append(name)
            self.trail_slots[slot] = created
            self.trail_sequence += 1
            self.last_trail_ns = stamp
        return True


def main():
    rospy.init_node('msim_gazebo_spoof_viz', anonymous=True)
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=45)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=45)
    rospy.wait_for_service('/gazebo/delete_model', timeout=45)
    viz = GazeboSpoofViz(rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel),
                        rospy.ServiceProxy('/gazebo/set_model_state', SetModelState),
                        rospy.ServiceProxy('/gazebo/delete_model', DeleteModel))
    rospy.Subscriber('/ground_truth/state', Odometry, viz.on_truth, queue_size=5)
    rospy.Subscriber('/gnss/spoof_detection', String, viz.on_detection, queue_size=5)
    ready_file = Path(os.environ['MSIM_GAZEBO_VIZ_READY'])
    rate = rospy.Rate(5)
    ready = False
    try:
        while not rospy.is_shutdown():
            if viz.update() and not ready:
                ready_file.write_text(json.dumps({'models': sorted(viz.models),
                                                  'legend': {'yellow': 'planned route',
                                                             'white': 'truth',
                                                             'blue': 'GNSS input',
                                                             'green': 'detector estimate',
                                                             'purple': 'injected offset reference (not GPS input)',
                                                             'red': 'alarm'}}))
                print('Gazebo markers ready: yellow=route, white=truth, blue=GNSS, green=estimate, purple=offset reference, red=alarm',
                      flush=True)
                ready = True
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    except rospy.ServiceException:
        if not rospy.is_shutdown():
            raise


if __name__ == '__main__':
    main()
