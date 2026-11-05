#!/usr/bin/env python3
"""Feed a single simulated GPS position stream into PX4 through MAVLink HIL_GPS.

This first online source is Gazebo truth with an optional fixed ENU bias. It is
deliberately labelled as such; SignalSim observations are still generated offline.
"""

import json
import math
import os
import time
from pathlib import Path

from pymavlink import mavutil
import rospy
from nav_msgs.msg import Odometry


ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT = 30.5312, 114.3601, 50.0
ANTENNA_ENU = (0.05, 0.0, 0.04)


def truth_to_gps(east, north, up, east_bias=0.0, north_bias=0.0):
    lat = math.radians(ORIGIN_LAT)
    sin_lat = math.sin(lat)
    a = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    prime_vertical = a / math.sqrt(1 - eccentricity_squared * sin_lat ** 2)
    meridian = a * (1 - eccentricity_squared) / (1 - eccentricity_squared * sin_lat ** 2) ** 1.5
    latitude = ORIGIN_LAT + math.degrees((north + north_bias) / (meridian + ORIGIN_ALT))
    longitude = ORIGIN_LON + math.degrees((east + east_bias) /
                                           ((prime_vertical + ORIGIN_ALT) * math.cos(lat)))
    return latitude, longitude, ORIGIN_ALT + up


def antenna_enu(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    x, y, z = ANTENNA_ENU
    # Rotate the body antenna lever arm into the Gazebo world ENU frame.
    tx = 2 * (q.y * z - q.z * y)
    ty = 2 * (q.z * x - q.x * z)
    tz = 2 * (q.x * y - q.y * x)
    offset = (x + q.w * tx + q.y * tz - q.z * ty,
              y + q.w * ty + q.z * tx - q.x * tz,
              z + q.w * tz + q.x * ty - q.y * tx)
    return p.x + offset[0], p.y + offset[1], p.z + offset[2]


def configure_px4(master):
    name = b'MAV_USEHILGPS'
    for _ in range(8):
        master.mav.param_set_send(master.target_system, 1, name, 1.0,
                                  mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            value = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.2)
            if value:
                param_id = value.param_id.decode() if isinstance(value.param_id, bytes) else value.param_id
            if value and param_id.rstrip('\x00') == name.decode() and round(value.param_value) == 1:
                print('PX4 accepted MAV_USEHILGPS=1', flush=True)
                return
    raise RuntimeError('PX4 did not acknowledge MAV_USEHILGPS=1')


def main():
    east_bias = float(os.environ.get('MSIM_GNSS_EAST_BIAS_M', '0'))
    north_bias = float(os.environ.get('MSIM_GNSS_NORTH_BIAS_M', '0'))
    if not all(math.isfinite(x) and abs(x) <= 1000 for x in (east_bias, north_bias)):
        raise ValueError('GNSS biases must be finite and within 1000 m')
    log_path = Path(os.environ['MSIM_GNSS_LOG'])
    # PX4 has a dedicated onboard MAVLink instance: 14280 local, 14030 remote.
    # The OFFBOARD patrol independently uses 14580/14540.
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14030', source_system=1,
                                         source_component=191)
    if master.wait_heartbeat(timeout=90) is None:
        raise RuntimeError('PX4 onboard MAVLink heartbeat did not arrive')
    configure_px4(master)
    rospy.init_node('msim_px4_online_gnss', anonymous=True)
    state = {'latest': None}

    def on_truth(msg):
        state['latest'] = msg

    rospy.Subscriber('/ground_truth/state', Odometry, on_truth, queue_size=10)
    previous = None
    next_stamp_ns = None
    with log_path.open('w', buffering=1) as log:
        while not rospy.is_shutdown():
            msg = state['latest']
            if msg is None:
                time.sleep(0.01)
                continue
            stamp_ns = msg.header.stamp.to_nsec()
            if next_stamp_ns is not None and stamp_ns < next_stamp_ns:
                time.sleep(0.01)
                continue
            if previous and stamp_ns <= previous[0]:
                time.sleep(0.01)
                continue
            east, north, up = antenna_enu(msg)
            latitude, longitude, altitude = truth_to_gps(
                east, north, up, east_bias, north_bias)
            if previous:
                elapsed = (stamp_ns - previous[0]) / 1e9
                ve = (east - previous[1]) / elapsed
                vn = (north - previous[2]) / elapsed
                vd = -(up - previous[3]) / elapsed
            else:
                ve = vn = vd = 0.0
            speed = math.hypot(vn, ve)
            cog = int(round(math.degrees(math.atan2(ve, vn)) % 360 * 100)) if speed > 0.05 else 65535
            master.mav.hil_gps_send(
                int(time.time() * 1e6), 3,
                int(round(latitude * 1e7)), int(round(longitude * 1e7)),
                int(round(altitude * 1e3)), 90, 180,
                min(65535, int(round(math.sqrt(ve * ve + vn * vn + vd * vd) * 100))),
                max(-32768, min(32767, int(round(vn * 100)))),
                max(-32768, min(32767, int(round(ve * 100)))),
                max(-32768, min(32767, int(round(vd * 100)))), cog, 16)
            log.write(json.dumps(dict(source='gazebo_truth', sim_time_ns=stamp_ns,
                                      lat_deg=latitude, lon_deg=longitude,
                                      alt_m=altitude, east_m=east, north_m=north,
                                      east_bias_m=east_bias, north_bias_m=north_bias),
                                 separators=(',', ':')) + '\n')
            previous = (stamp_ns, east, north, up)
            next_stamp_ns = stamp_ns + 200_000_000
            time.sleep(0.005)


if __name__ == '__main__':
    main()
