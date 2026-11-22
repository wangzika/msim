#!/usr/bin/env python3
"""Create a PX4/Gazebo model and matching FAST-LIO extrinsic for a tilt trial."""
import argparse
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'adapters/ros1_noetic/fast_glio_uav'))
from prepare_px4_assets import prepare_model, prepare_world  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--case-dir', type=Path, required=True)
    ap.add_argument('--forward-down-deg', type=float, required=True)
    ap.add_argument('--px4-root', type=Path,
                    default=ROOT / 'build/deps/PX4-Autopilot')
    ap.add_argument('--fast-glio-root', type=Path,
                    default=Path('/home/wzb/FAST_GLIO'))
    args = ap.parse_args()
    if not 0 <= args.forward_down_deg <= 20:
        ap.error('forward-down angle must be 0–20 degrees')
    args.case_dir.mkdir(parents=True, exist_ok=True)
    # With this corridor flight, +Y is forward and -roll points +Y rays down.
    roll_deg = -args.forward_down_deg
    model = args.px4_root / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf'
    world = args.fast_glio_root / 'simulation/uav_simulator/worlds/urban_canyon.world'
    prepare_model(model, args.case_dir / 'iris_msim.sdf',
                  lidar_roll_deg=roll_deg)
    prepare_world(world, args.case_dir / 'urban_canyon_msim.world')
    theta = math.radians(roll_deg)
    c, s = math.cos(theta), math.sin(theta)
    matrix = [1.0, 0.0, 0.0,
              0.0, c, -s,
              0.0, s, c]
    config = (ROOT / 'adapters/ros1_noetic/fast_glio_uav/fast_lio_velodyne_benchmark.yaml').read_text()
    old = 'extrinsic_R: [1, 0, 0,\n                  0, 1, 0,\n                  0, 0, 1]'
    if config.count(old) != 1:
        raise RuntimeError('FAST-LIO benchmark extrinsic_R format changed')
    new = 'extrinsic_R: [' + ', '.join(f'{value:.10f}' for value in matrix) + ']'
    (args.case_dir / 'fast_lio_tilt.yaml').write_text(config.replace(old, new))
    manifest = dict(forward_down_deg=args.forward_down_deg,
                    gazebo_sensor_roll_deg=roll_deg,
                    lidar_to_imu_rotation_row_major=matrix,
                    lidar_to_imu_translation_m=[0.0, 0.0, 0.15],
                    model=str(args.case_dir / 'iris_msim.sdf'),
                    fast_lio_config=str(args.case_dir / 'fast_lio_tilt.yaml'))
    (args.case_dir / 'tilt.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
