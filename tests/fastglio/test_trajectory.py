import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from apps.fastglio.trajectory import antenna_state, validate


def trajectory():
    return {'schema_version': 1, 'frame': 'map', 'antenna_offset_base_m': [1.0, 0.0, 0.0],
            'samples': [
                {'sim_time_ns': 0, 'position_ecef_m': [10.0, 0.0, 0.0],
                 'velocity_ecef_mps': [2.0, 0.0, 0.0],
                 'orientation_earth_base_xyzw': [0.0, 0.0, 0.0, 1.0],
                 'angular_velocity_body_radps': [0.0, 0.0, 1.0]},
                {'sim_time_ns': 10_000_000, 'position_ecef_m': [10.02, 0.0, 0.0],
                 'velocity_ecef_mps': [2.0, 0.0, 0.0],
                 'orientation_earth_base_xyzw': [0.0, 0.0, 0.0, 1.0],
                 'angular_velocity_body_radps': [0.0, 0.0, 1.0]}]}


class TrajectoryTest(unittest.TestCase):
    def test_interpolation_and_lever_velocity(self):
        data = validate(trajectory())
        position, velocity = antenna_state(data, 5_000_000)
        self.assertEqual(position, [11.01, 0.0, 0.0])
        self.assertEqual(velocity, [2.0, 1.0, 0.0])

    def test_reject_time_gap_and_extrapolation(self):
        data = trajectory()
        data['samples'][1]['sim_time_ns'] = 30_000_000
        with self.assertRaises(ValueError):
            validate(data)
        with self.assertRaises(ValueError):
            antenna_state(trajectory(), 11_000_000)


if __name__ == '__main__':
    unittest.main()
