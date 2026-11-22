import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'adapters/ros1_noetic/msim_sensors'))
from sensor_profiles import resolve, validate, LIDARS, MID360_PATTERN

spec = importlib.util.spec_from_file_location('assets', ROOT / 'adapters/ros1_noetic/fast_glio_uav/prepare_px4_assets.py')
assets = importlib.util.module_from_spec(spec)
spec.loader.exec_module(assets)


class SensorProfilesTest(unittest.TestCase):
    def test_presets_and_custom_override(self):
        for name, expected in LIDARS.items():
            p = resolve(lidar=name)
            self.assertEqual(p['lidar']['vertical_samples'], expected['vertical_samples'])
            self.assertEqual(p['lidar']['timing'], 'rolling')
        p = resolve(imu='low-noise', gps='native', timing='snapshot')
        self.assertEqual(p['gps']['position_sigma_xy_m'], 0)
        self.assertAlmostEqual(p['imu_ros']['gyro_noise_density'], resolve()['imu_ros']['gyro_noise_density']*.2)
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)/'custom.json'
            p['lidar']['horizontal_samples'] = 120
            f.write_text(json.dumps(p))
            self.assertEqual(resolve(f)['lidar']['horizontal_samples'], 120)

    def test_invalid_model_parameters_fail_before_launch(self):
        for section, key, value in [('lidar', 'rate_hz', 0), ('lidar', 'vertical_samples', 1),
                                    ('lidar', 'timing', 'fake'), ('gps', 'position_sigma_xy_m', -1),
                                    ('gps', 'reported_variance_xy_m2', float('nan'))]:
            p = resolve(); p[section][key] = value
            with self.assertRaises(ValueError):
                validate(p)

    def test_mid360_uses_pinned_recorded_pattern_not_uniform_beams(self):
        p=resolve(lidar='mid360')
        self.assertEqual(p['lidar']['scan_pattern'],'livox_csv')
        self.assertEqual(p['lidar']['point_rate_hz'],200000)
        self.assertEqual(p['lidar']['vertical_samples'],4)
        self.assertEqual(p['lidar']['range_min_m'],.1)
        self.assertTrue(MID360_PATTERN.is_file())
        p['lidar']['pattern_sha256']='bad'
        with self.assertRaises(ValueError):validate(p)

    def test_switch_back_from_livox_clears_pattern_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            f=Path(tmp)/'mid360.json';f.write_text(json.dumps(resolve(lidar='mid360')))
            p=resolve(f,lidar='vlp16')
            self.assertEqual(p['lidar']['scan_pattern'],'uniform')
            self.assertNotIn('pattern_file',p['lidar'])

    def test_generated_model_has_one_native_gps_route_and_scanner(self):
        source = ROOT/'build/deps/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf'
        if not source.exists():
            self.skipTest('PX4 dependencies not installed')
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp)/'vehicle.sdf'
            assets.prepare_model(source, target, online_gnss=True, sensor_profile=resolve(lidar='hdl32'))
            model = ET.parse(target).getroot().find('model')
            self.assertIsNone(model.find("include[name='gps0']"))
            gps = model.find("model[@name='gps0']")
            self.assertEqual(gps.findtext('.//plugin/topic'), 'msim_gps_raw')
            self.assertIsNotNone(model.find("joint[@name='gps0_joint']"))
            self.assertEqual(len(model.findall("plugin[@name='msim_gps_relay']")), 1)
            scanner = model.find("plugin[@name='msim_scanning_lidar']")
            self.assertEqual(scanner.findtext('vertical_samples'), '32')
            self.assertEqual(scanner.findtext('timing'), 'rolling')
            self.assertIsNone(model.find(".//sensor[@name='msim_lidar']"))


if __name__ == '__main__':
    unittest.main()
