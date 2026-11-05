#!/usr/bin/env python3
"""Check that a PX4 capture includes a real armed OFFBOARD flight."""

import argparse
import json
import math
from pathlib import Path

from pyulog import ULog


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', type=Path, required=True)
    ap.add_argument('--online-gnss', action='store_true')
    args = ap.parse_args()
    trajectory = json.loads((args.run / 'trajectory.json').read_text())['samples']
    first = trajectory[0]['position_ecef_m']
    max_displacement = max(math.dist(first, s['position_ecef_m']) for s in trajectory)
    logs = sorted((args.run / 'ros-home/log').rglob('*.ulg'))
    if len(logs) != 1:
        ap.error(f'expected one PX4 ULog in {args.run / "ros-home/log"}; found {len(logs)}')
    ulog = ULog(str(logs[0]), message_name_filter_list=[
        'vehicle_status', 'vehicle_local_position', 'sensor_gps'])
    data = {dataset.name: dataset.data for dataset in ulog.data_list}
    for topic in ('vehicle_status', 'vehicle_local_position', 'sensor_gps'):
        if topic not in data:
            ap.error(f'PX4 ULog has no {topic}')
    status = data['vehicle_status']
    positions = data['vehicle_local_position']
    gps = data['sensor_gps']
    max_climb = float(positions['z'][0] - min(positions['z']))
    armed = any(status['arming_state'] == 2)
    offboard = any(status['nav_state'] == 14)
    gps_count = len(gps['timestamp'])
    max_satellites = int(max(gps['satellites_used'])) if gps_count else 0
    injection_match_fraction = None
    if args.online_gnss:
        injected = [json.loads(line) for line in
                    (args.run / 'injected-gps.jsonl').read_text().splitlines()]
        if not injected or any(row['source'] != 'gazebo_truth' for row in injected):
            ap.error('online GNSS injection log missing or source unknown')
        injected_coordinates = {(round(row['lat_deg'] * 1e7),
                                 round(row['lon_deg'] * 1e7)) for row in injected}
        matched = sum((round(lat * 1e7), round(lon * 1e7)) in injected_coordinates
                      for lat, lon in zip(gps['latitude_deg'], gps['longitude_deg']))
        injection_match_fraction = matched / gps_count if gps_count else 0.0
    ok = (armed and offboard and max_displacement >= 5 and max_climb >= 3
          and gps_count >= 5 and max_satellites >= 4 and
          (not args.online_gnss or injection_match_fraction >= 0.8))
    report = dict(pass_flight_gate=bool(ok), armed=bool(armed), offboard=bool(offboard),
                  max_displacement_m=max_displacement, max_climb_m=max_climb,
                  px4_gps_samples=gps_count, max_px4_gps_satellites=max_satellites,
                  online_gnss=args.online_gnss,
                  injection_match_fraction=injection_match_fraction,
                  px4_ulog=str(logs[0]))
    (args.run / 'px4-flight-report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if not ok:
        raise SystemExit('PX4 flight gate failed; inspect px4-flight-report.json and controller.log')


if __name__ == '__main__':
    main()
