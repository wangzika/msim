#!/usr/bin/env python3
"""Monitor the native Gazebo GPS relay and publish spoof-detection diagnostics.

The default plugin mode preserves native GPS noise and latency. The historical
truth-based MAVLink HIL_GPS source remains available for legacy runs.
"""

import json
import math
import os
import time
import threading
from collections import deque
from pathlib import Path

from pymavlink import mavutil
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

from sensor_sync import PastSampleBuffer
from scene_profile import from_environment
from spoof_detector import PlanarInnovationDetector, horizontal_acceleration


ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT = from_environment()['origin_lla_deg_m']
ANTENNA_ENU = (0.05, 0.0, 0.04)
MAX_IMU_AGE_NS = 50_000_000
MAX_LIDAR_AGE_NS = 200_000_000


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


def native_monitor():
    """Log/detect the plugin's final measurements; never inject a second GPS."""
    rospy.init_node('msim_native_gnss_monitor', anonymous=True)
    lock = threading.Lock()
    truths, pending = deque(maxlen=1000), deque(maxlen=100)
    imus, lidars = PastSampleBuffer(), PastSampleBuffer()
    detector = PlanarInnovationDetector()
    output = rospy.Publisher('/gnss/spoof_detection', String, queue_size=20)

    def truth(msg):
        with lock:
            truths.append(msg)

    def imu(msg):
        value = horizontal_acceleration(msg)
        if value is not None:
            with lock:
                imus.add(msg.header.stamp.to_nsec(), value)

    def lidar(msg):
        with lock:
            lidars.add(msg.header.stamp.to_nsec(), dict(point_count=msg.width * msg.height,
                                                        frame_id=msg.header.frame_id))

    def sample(msg):
        with lock:
            pending.append(json.loads(msg.data))

    subs = [rospy.Subscriber('/ground_truth/state', Odometry, truth, queue_size=200),
            rospy.Subscriber('/imu/data', Imu, imu, queue_size=200),
            rospy.Subscriber('/velodyne_points', PointCloud2, lidar, queue_size=20),
            rospy.Subscriber('/msim/gps_samples', String, sample, queue_size=20)]
    with Path(os.environ['MSIM_GNSS_LOG']).open('w', buffering=1) as log:
        while not rospy.is_shutdown():
            with lock:
                row = pending.popleft() if pending else None
                history = list(truths)
            if row is None:
                time.sleep(.005)
                continue
            stamp = row['sim_time_ns']
            if not history:
                continue
            reference = min(history, key=lambda m: abs(m.header.stamp.to_nsec()-stamp))
            if abs(reference.header.stamp.to_nsec()-stamp) > 30_000_000:
                continue
            east, north, up = antenna_enu(reference)
            with lock:
                im = imus.match(stamp, MAX_IMU_AGE_NS)
                li = lidars.match(stamp, MAX_LIDAR_AGE_NS)
            detection = detector.step(stamp, row['gps_east_m'], row['gps_north_m'], im['value'])
            meta = li['value']
            row.update(time_base='gazebo_sim_time', east_m=east, north_m=north, up_m=up,
                       east_bias_m=0., north_bias_m=0., truth_stamp_ns=reference.header.stamp.to_nsec(),
                       imu_available=im['value'] is not None, imu_sync_status=im['status'],
                       imu_stamp_ns=im['stamp_ns'], imu_age_ms=im['age_ms'],
                       lidar_sync_status=li['status'], lidar_stamp_ns=li['stamp_ns'],
                       lidar_age_ms=li['age_ms'], lidar_point_count=meta['point_count'] if meta else None,
                       lidar_frame_id=meta['frame_id'] if meta else None,
                       **{k: v for k, v in detection.items() if k != 'sim_time_ns'})
            payload = json.dumps(row, separators=(',', ':'))
            output.publish(String(data=payload)); log.write(payload+'\n')
    for sub in subs:
        sub.unregister()


def main():
    if os.environ.get('MSIM_GNSS_SOURCE') == 'gazebo_plugin':
        native_monitor()
        return
    east_bias = float(os.environ.get('MSIM_GNSS_EAST_BIAS_M', '0'))
    north_bias = float(os.environ.get('MSIM_GNSS_NORTH_BIAS_M', '0'))
    if not all(math.isfinite(x) and abs(x) <= 1000 for x in (east_bias, north_bias)):
        raise ValueError('GNSS biases must be finite and within 1000 m')
    spoof_start_s = float(os.environ.get('MSIM_SPOOF_START_S', 'inf'))
    spoof_east_rate = float(os.environ.get('MSIM_SPOOF_EAST_RATE_MPS', '0'))
    spoof_north_rate = float(os.environ.get('MSIM_SPOOF_NORTH_RATE_MPS', '0'))
    if not all(math.isfinite(x) and abs(x) <= 10 for x in (spoof_east_rate, spoof_north_rate)):
        raise ValueError('spoof drift rates must be finite and within 10 m/s')
    if math.isnan(spoof_start_s) or spoof_start_s < 0:
        raise ValueError('spoof start must be non-negative')
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
    imu_samples = PastSampleBuffer()
    lidar_samples = PastSampleBuffer()
    lock = threading.Lock()
    detector = PlanarInnovationDetector()
    detection_pub = rospy.Publisher('/gnss/spoof_detection', String, queue_size=10)
    loose_gnss_pub = rospy.Publisher('/msim/spoofed_gnss_odom', Odometry,
                                    queue_size=10)

    def on_truth(msg):
        with lock:
            state['latest'] = msg

    def on_imu(msg):
        acceleration = horizontal_acceleration(msg)
        if acceleration is not None:
            with lock:
                imu_samples.add(msg.header.stamp.to_nsec(), acceleration)

    def on_lidar(msg):
        # Keep only scan metadata; the full point cloud is already in sensors.bag.
        with lock:
            lidar_samples.add(msg.header.stamp.to_nsec(),
                              dict(point_count=msg.width * msg.height,
                                   frame_id=msg.header.frame_id))

    rospy.Subscriber('/ground_truth/state', Odometry, on_truth, queue_size=10)
    rospy.Subscriber('/imu/data', Imu, on_imu, queue_size=50)
    rospy.Subscriber('/velodyne_points', PointCloud2, on_lidar, queue_size=10)
    previous = None
    next_stamp_ns = None
    first_stamp_ns = None
    with log_path.open('w', buffering=1) as log:
        while not rospy.is_shutdown():
            with lock:
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
            with lock:
                imu_match = imu_samples.match(stamp_ns, MAX_IMU_AGE_NS)
                lidar_match = lidar_samples.match(stamp_ns, MAX_LIDAR_AGE_NS)
            east, north, up = antenna_enu(msg)
            if first_stamp_ns is None:
                first_stamp_ns = stamp_ns
            elapsed_s = (stamp_ns - first_stamp_ns) / 1e9
            attack_elapsed_s = max(0.0, elapsed_s - spoof_start_s)
            attack_east_m = attack_elapsed_s * spoof_east_rate
            attack_north_m = attack_elapsed_s * spoof_north_rate
            latitude, longitude, altitude = truth_to_gps(
                east, north, up, east_bias + attack_east_m,
                north_bias + attack_north_m)
            if previous:
                elapsed = (stamp_ns - previous[0]) / 1e9
                ve = (east - previous[1]) / elapsed
                vn = (north - previous[2]) / elapsed
                vd = -(up - previous[3]) / elapsed
            else:
                ve = vn = vd = 0.0
            speed = math.hypot(vn, ve)
            cog = int(round(math.degrees(math.atan2(ve, vn)) % 360 * 100)) if speed > 0.05 else 65535
            mavlink_time_utc_us = int(time.time() * 1e6)
            master.mav.hil_gps_send(
                mavlink_time_utc_us, 3,
                int(round(latitude * 1e7)), int(round(longitude * 1e7)),
                int(round(altitude * 1e3)), 90, 180,
                min(65535, int(round(math.sqrt(ve * ve + vn * vn + vd * vd) * 100))),
                max(-32768, min(32767, int(round(vn * 100)))),
                max(-32768, min(32767, int(round(ve * 100)))),
                max(-32768, min(32767, int(round(vd * 100)))), cog, 16)
            # FAST_GLIO receives the same spoofed antenna position as PX4, but
            # as a stamped ENU position solution rather than a raw RF signal.
            loose_gnss = Odometry()
            loose_gnss.header.stamp = msg.header.stamp
            loose_gnss.header.frame_id = 'msim_enu'
            loose_gnss.child_frame_id = 'gnss_antenna'
            loose_gnss.pose.pose.position.x = east + east_bias + attack_east_m
            loose_gnss.pose.pose.position.y = north + north_bias + attack_north_m
            loose_gnss.pose.pose.position.z = up
            loose_gnss.pose.pose.orientation.w = 1.0
            loose_gnss.pose.covariance[0] = loose_gnss.pose.covariance[7] = 0.49
            loose_gnss.pose.covariance[14] = 4.0
            loose_gnss_pub.publish(loose_gnss)
            acceleration = imu_match['value']
            detection = detector.step(stamp_ns, east + east_bias + attack_east_m,
                                      north + north_bias + attack_north_m, acceleration)
            lidar_meta = lidar_match['value']
            record = dict(source='gazebo_truth', sim_time_ns=stamp_ns,
                          time_base='gazebo_sim_time',
                          mavlink_time_utc_us=mavlink_time_utc_us,
                          lat_deg=latitude, lon_deg=longitude,
                          alt_m=altitude, up_m=up,
                          east_m=east, north_m=north,
                          east_bias_m=east_bias, north_bias_m=north_bias,
                          attack_east_m=attack_east_m,
                          attack_north_m=attack_north_m,
                          attack_active=attack_elapsed_s > 0,
                          imu_available=acceleration is not None,
                          imu_sync_status=imu_match['status'],
                          imu_stamp_ns=imu_match['stamp_ns'],
                          imu_age_ms=imu_match['age_ms'],
                          lidar_sync_status=lidar_match['status'],
                          lidar_stamp_ns=lidar_match['stamp_ns'],
                          lidar_age_ms=lidar_match['age_ms'],
                          lidar_point_count=(lidar_meta['point_count']
                                             if lidar_meta else None),
                          lidar_frame_id=(lidar_meta['frame_id']
                                          if lidar_meta else None),
                          **{key: value for key, value in detection.items()
                             if key != 'sim_time_ns'})
            payload = json.dumps(record, separators=(',', ':'))
            detection_pub.publish(String(data=payload))
            log.write(payload + '\n')
            previous = (stamp_ns, east, north, up)
            next_stamp_ns = stamp_ns + 200_000_000
            time.sleep(0.005)


if __name__ == '__main__':
    main()
