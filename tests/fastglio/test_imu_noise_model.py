import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'adapters/ros1_noetic/msim_sensors'
sys.path.insert(0, str(PACKAGE / 'scripts'))

from imu_noise_model import ImuConfig, ImuNoiseModel


class ImuNoiseModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = json.loads((PACKAGE / 'config/sensor_profile.json').read_text())
        cls.config = ImuConfig.from_dict(cls.profile['imu_ros'])

    def test_repeatability_and_unit_quaternion(self):
        models = [ImuNoiseModel(self.config), ImuNoiseModel(self.config)]
        runs = [[model.sample(i * 0.005, (0, 0, 0), (0, 0, 9.81), (0, 0, 0, 1))
                 for i in range(20)] for model in models]
        self.assertEqual(runs[0], runs[1])
        for sample in runs[0]:
            self.assertAlmostEqual(sum(v * v for v in sample['orientation']), 1.0, 12)

    def test_axes_have_distinct_noise_covariance_and_bias_drifts(self):
        model = ImuNoiseModel(self.config)
        first = model.sample(1.0, (0, 0, 0), (0, 0, 9.81), (0, 0, 0, 1))
        later = model.sample(1.005, (0, 0, 0), (0, 0, 9.81), (0, 0, 0, 1))
        self.assertAlmostEqual(first['gyro_variance'],
                               self.config.gyro_noise_density ** 2 / 0.005, places=12)
        self.assertAlmostEqual(first['accel_variance'],
                               self.config.accel_noise_density ** 2 / 0.005, places=12)
        self.assertNotEqual(first['gyro_variance'], first['accel_variance'])
        self.assertNotEqual(first['gyro_bias'], later['gyro_bias'])
        self.assertNotEqual(first['accel_bias'], later['accel_bias'])
        self.assertTrue(all(math.isfinite(v) for v in later['angular_velocity']))

    def test_invalid_time_and_profile_are_rejected(self):
        model = ImuNoiseModel(self.config)
        model.sample(1.0, (0, 0, 0), (0, 0, 9.81), (0, 0, 0, 1))
        with self.assertRaises(ValueError):
            model.sample(1.0, (0, 0, 0), (0, 0, 9.81), (0, 0, 0, 1))
        invalid = dict(self.profile['imu_ros'], gyro_bias_correlation_time_s=0)
        with self.assertRaises(ValueError):
            ImuConfig.from_dict(invalid)


if __name__ == '__main__':
    unittest.main()
