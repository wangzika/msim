#!/usr/bin/env python3
"""Fly PX4 Iris in the selected scene using MAVLink offboard setpoints."""

import sys
import time
import os
import json
from pathlib import Path

from pymavlink import mavutil
from scene_profile import from_environment


RATE_HZ = 10
HEIGHT_M = from_environment()['flight_height_m']
# Relative to the configured spawn; MAVLink LOCAL_NED = north, east.
WAYPOINTS = ((0, 0, 15), (15, 4, 30), (30, 4, 30),
             (45, -4, 35), (50, -4, 30), (30, 0, 30), (0, 0, 35))
ROUTE_PLAN = Path(os.environ.get('MSIM_ROUTE_PLAN',
                                 str(Path(__file__).with_name('route_plan.json'))))


def main():
    pattern = os.environ.get('MSIM_PX4_FLIGHT_PATTERN', 'patrol')
    if pattern not in ('patrol', 'hover', 'route'):
        raise ValueError(f'unknown PX4 flight pattern: {pattern}')
    if pattern == 'route':
        route = json.loads(ROUTE_PLAN.read_text())
        waypoints = tuple((point['north_m'], point['east_m'], point['hold_s'])
                          for point in route)
    else:
        waypoints = ((0, 0, 3600),) if pattern == 'hover' else WAYPOINTS
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

    print(f'PX4 flight pattern: {pattern}', flush=True)
    route_clock=os.environ.get('MSIM_ROUTE_CLOCK','wall')
    if route_clock not in ('wall','sim'):raise ValueError('route clock must be wall or sim')
    def flight_time():
        if route_clock=='wall':return time.monotonic()
        # Drain MAVLink receive data; PX4 boot time advances with the SITL clock.
        for _ in range(200):
            if master.recv_match(blocking=False) is None:break
        position=master.messages.get('LOCAL_POSITION_NED')
        if position is None:raise RuntimeError('PX4 simulation clock unavailable')
        return position.time_boot_ms/1000.0
    print(f'PX4 route dwell clock: {route_clock}',flush=True)
    while True:
        for north, east, seconds in waypoints:
            begin=flight_time()
            print(f'patrol setpoint N={north} E={east} z={HEIGHT_M}, hold={seconds}s, clock={begin:.3f}', flush=True)
            until = begin + seconds
            while flight_time() < until:
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
