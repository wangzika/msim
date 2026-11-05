#!/usr/bin/env python3
"""Freeze the ROS 1 UAV SignalSim configuration for a captured flight."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TEMPLATE = ROOT / 'scenarios/fastglio_signalsim.json'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--duration-s', type=int, required=True)
    parser.add_argument('--scenario', type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    if args.duration_s < 2:
        parser.error('duration must be at least 2 seconds')
    report = json.loads((run / 'capture-report.json').read_text())
    if report['duration_s'] != args.duration_s:
        parser.error('capture duration differs from requested duration')
    existing = run / 'scenario.json'
    manifest_path = run / 'signalsim/manifest.json'
    if args.scenario:
        scenario_path = args.scenario.resolve()
    elif manifest_path.is_file():
        expected = json.loads(manifest_path.read_text())['scenario_sha256']
        matching = [path for path in run.glob('scenario*.json') if digest(path) == expected]
        if not matching:
            parser.error('existing SignalSim output has no matching scenario JSON; pass --scenario')
        scenario_path = matching[0]
    elif existing.is_file() and json.loads(existing.read_text())['gnss'].get('backend') == 'signalsim_observables':
        scenario_path = existing
    else:
        scenario_path = run / ('scenario-signalsim.json' if existing.is_file() else 'scenario.json')
        if not scenario_path.exists():
            config = json.loads(TEMPLATE.read_text())
            if existing.is_file():
                old = json.loads(existing.read_text())
                config['time']['utc_start'] = old['time']['utc_start']
                config['geodesy']['origin_lla_deg_m'] = old['geodesy']['origin_lla_deg_m']
                if 'base_enu_m' in old.get('receivers', {}):
                    config['receivers']['base_enu_m'] = old['receivers']['base_enu_m']
                for key in ('urban_boxes_enu_m', 'urban_reflection_coefficient'):
                    if key in old.get('gnss', {}):
                        config['gnss'][key] = old['gnss'][key]
            config['run_id'] = run.name[:96]
            config['time']['duration_s'] = args.duration_s
            config['gnss']['ephemeris'] = str(ROOT / 'build/deps/SignalSim/EphData/BRDC00IGS_R_20211700000_01D_MN.rnx')
            config['gnss']['leap_seconds_source'] = str(ROOT / 'data/time/leap-seconds.list')
            config['outputs']['root'] = str(run / 'signalsim')
            scenario_path.write_text(json.dumps(config, indent=2) + '\n')
    config = json.loads(scenario_path.read_text())
    if (config['gnss'].get('backend') != 'signalsim_observables' and
            not (manifest_path.is_file() and
                 json.loads(manifest_path.read_text())['scenario_sha256'] == digest(scenario_path))):
        parser.error('scenario must use signalsim_observables')
    if config['time']['duration_s'] != args.duration_s or config['time']['gnss_rate_hz'] != 5:
        parser.error('scenario GNSS schedule differs from capture')
    if report['origin_lla_deg_m'] != config['geodesy']['origin_lla_deg_m']:
        parser.error('GNSS and truth geodetic origins differ')
    print(scenario_path)


if __name__ == '__main__':
    main()
