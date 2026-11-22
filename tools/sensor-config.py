#!/usr/bin/env python3
"""List or resolve sensor presets without starting a simulation."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'adapters/ros1_noetic/msim_sensors'))
from sensor_profiles import resolve, LIDARS, IMUS, GPS


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--list', action='store_true')
    p.add_argument('--sensor-profile', type=Path)
    p.add_argument('--lidar-model', choices=LIDARS)
    p.add_argument('--imu-model', choices=IMUS)
    p.add_argument('--gps-model', choices=GPS)
    p.add_argument('--lidar-timing', choices=('snapshot', 'rolling'))
    p.add_argument('--output', type=Path)
    a = p.parse_args()
    if a.list:
        print('LiDAR: vlp16, hdl32, os1-64 (uniform beams); mid360 (official Livox CSV scan pattern)')
        print('IMU: ' + ', '.join(IMUS) + ' (research error presets)')
        print('GPS: ' + ', '.join(GPS))
        print('LiDAR timing: rolling, snapshot; mid360 uses native Livox FAST-LIO input automatically')
        return
    try:
        profile = resolve(a.sensor_profile, a.lidar_model, a.imu_model, a.gps_model, a.lidar_timing)
    except (ValueError, KeyError) as exc:
        p.error(str(exc))
    text = json.dumps(profile, indent=2) + '\n'
    if a.output:
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text(text)
    else:
        print(text, end='')


if __name__ == '__main__':
    main()
