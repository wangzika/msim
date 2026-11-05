import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'adapters/ros1_noetic/fast_glio_uav/prepare_run.py'
ORIGIN = [30.5312, 114.3601, 50.0]


class PrepareRunTest(unittest.TestCase):
    def test_new_capture_gets_standalone_signalsim_scenario(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / 'flight'
            run.mkdir()
            (run / 'capture-report.json').write_text(json.dumps({
                'duration_s': 2, 'origin_lla_deg_m': ORIGIN}))
            result = subprocess.check_output([sys.executable, str(SCRIPT),
                                              '--run', str(run), '--duration-s', '2'], text=True)
            scenario = Path(result.strip())
            config = json.loads(scenario.read_text())
            self.assertEqual(scenario, run / 'scenario.json')
            self.assertEqual(config['gnss']['backend'], 'signalsim_observables')
            self.assertEqual(config['time']['duration_s'], 2)
            self.assertEqual(config['receivers']['base_enu_m'], [0.0, -100.0, 10.0])
            self.assertEqual(config['gnss']['urban_reflection_coefficient'], 0.35)

    def test_existing_output_uses_matching_historical_scenario(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / 'flight'
            (run / 'signalsim').mkdir(parents=True)
            (run / 'capture-report.json').write_text(json.dumps({
                'duration_s': 2, 'origin_lla_deg_m': ORIGIN}))
            old = {'gnss': {'backend': 'synthetic_geometry'},
                   'time': {'duration_s': 2, 'gnss_rate_hz': 5},
                   'geodesy': {'origin_lla_deg_m': ORIGIN}}
            path = run / 'scenario-historical.json'
            path.write_text(json.dumps(old))
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            (run / 'signalsim/manifest.json').write_text(json.dumps({'scenario_sha256': digest}))
            result = subprocess.check_output([sys.executable, str(SCRIPT),
                                              '--run', str(run), '--duration-s', '2'], text=True)
            self.assertEqual(Path(result.strip()), path)


if __name__ == '__main__':
    unittest.main()
