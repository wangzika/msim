#!/usr/bin/env python3
"""Fly PX4 Iris through the urban corridor using MAVLink offboard setpoints."""

import sys
import time

from pymavlink import mavutil


RATE_HZ = 10
HEIGHT_M = 8.0
# Relative to the spawn point (Gazebo ENU 0, -20); MAVLink LOCAL_NED = north, east.
WAYPOINTS = ((0, 0, 15), (15, 4, 30), (30, 4, 30),
             (45, -4, 35), (50, -4, 30), (30, 0, 30), (0, 0, 35))


def main():
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14540', source_system=255)
    master.wait_heartbeat(timeout=90)
    print('PX4 MAVLink heartbeat received', flush=True)
    local = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=30)
    if local is None:
        raise RuntimeError('PX4 local position unavailable; check GPS and EKF readiness')
    origin = (local.x, local.y, local.z)
    mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

    def send(north, east):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            origin[0] + north, origin[1] + east, origin[2] - HEIGHT_M,
            0, 0, 0, 0, 0, 0, 0, 0)

    for _ in range(30):
        send(0, 0)
        time.sleep(1 / RATE_HZ)
    deadline = time.monotonic() + 90
    next_command = 0
    while time.monotonic() < deadline:
        send(0, 0)
        if time.monotonic() >= next_command:
            master.set_mode('OFFBOARD')
            master.arducopter_arm()
            next_command = time.monotonic() + 3
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and \
                mavutil.mode_string_v10(msg) == 'OFFBOARD':
            print('PX4 armed in OFFBOARD; beginning patrol', flush=True)
            break
        time.sleep(1 / RATE_HZ)
    else:
        raise RuntimeError('PX4 did not arm in OFFBOARD within 90 s; inspect estimator and controller log')

    while True:
        for north, east, seconds in WAYPOINTS:
            print(f'patrol setpoint N={north} E={east} z={HEIGHT_M}, hold={seconds}s', flush=True)
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                send(north, east)
                time.sleep(1 / RATE_HZ)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('PX4 patrol stopped after capture', flush=True)
    except Exception as exc:
        print(f'PX4 patrol failed: {exc}', file=sys.stderr, flush=True)
        raise
