"""Shared scene origin, spawn, camera and flight settings for PX4 adapters."""
import argparse
import json
import math
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT = dict(schema_version=1, name='default',
               origin_lla_deg_m=[30.5312, 114.3601, 50.0],
               spawn_enu_m=[0., -20., 0.], flight_height_m=8.,
               camera_pose=[0., -48., 24., 0., .55, 1.57],
               route_plan=str(HERE/'route_plan.json'))


def load_scene_profile(path=None):
    profile = dict(DEFAULT)
    if path:
        path = Path(path).resolve()
        profile.update(json.loads(path.read_text()))
        for key in ('route_plan', 'asset_manifest', 'resource_path'):
            if profile.get(key):
                p = Path(profile[key])
                profile[key] = str((p if p.is_absolute() else path.parent/p).resolve())
    if profile['schema_version'] != 1:
        raise ValueError('Unsupported scene profile schema')
    for key, count in [('origin_lla_deg_m', 3), ('spawn_enu_m', 3), ('camera_pose', 6)]:
        values = profile[key]
        if len(values) != count or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                       or not math.isfinite(v) for v in values):
            raise ValueError('Invalid '+key)
    lat, lon, _ = profile['origin_lla_deg_m']
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError('Scene geographic origin is out of range')
    height = profile['flight_height_m']
    if isinstance(height, bool) or not isinstance(height, (float, int)) or not math.isfinite(height) or height <= 0:
        raise ValueError('Flight height must be finite and positive')
    return profile


def from_environment():
    return load_scene_profile(os.environ.get('MSIM_SCENE_PROFILE'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', type=Path)
    ap.add_argument('--output', type=Path)
    ap.add_argument('--values', action='store_true')
    args = ap.parse_args()
    profile = load_scene_profile(args.profile)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(profile, indent=2)+'\n')
    if args.values:
        for value in [*profile['origin_lla_deg_m'], *profile['spawn_enu_m'],
                      profile['flight_height_m'], profile['route_plan'],
                      profile.get('resource_path', '')]:
            print(value)
    elif not args.output:
        print(json.dumps(profile, indent=2))


if __name__ == '__main__':
    main()
