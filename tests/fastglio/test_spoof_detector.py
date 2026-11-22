import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       'adapters/ros1_noetic/fast_glio_uav'))
from spoof_detector import CHI2_2_9999, PlanarInnovationDetector


class SpoofDetectorTest(unittest.TestCase):
    def test_stationary_then_position_attack(self):
        detector = PlanarInnovationDetector()
        baseline = [detector.step(i * 200_000_000, 0., 0., (0., 0.))
                    for i in range(30)]
        self.assertTrue(all(row['detector_status'] == 'normal' for row in baseline[1:]))
        attack = detector.step(30 * 200_000_000, 8., 0., (0., 0.))
        self.assertEqual(attack['detector_status'], 'alarm')
        self.assertGreater(attack['innovation_chi2'], CHI2_2_9999)
        self.assertLess(abs(attack['estimate_east_m']), 1.)

    def test_motion_with_matching_imu_and_gps(self):
        detector = PlanarInnovationDetector()
        rows = [detector.step(i * 200_000_000, 0.5 * (i * 0.2)**2,
                              0., (1., 0.)) for i in range(40)]
        self.assertFalse(any(row['detector_status'] == 'alarm' for row in rows))
        self.assertAlmostEqual(rows[-1]['estimate_east_m'],
                               0.5 * (39 * .2)**2, delta=1.)

    def test_slow_drift_against_inertial_reference(self):
        detector = PlanarInnovationDetector()
        rows = [detector.step(i * 200_000_000, max(0., i * .2 - 18.) * 2.,
                              0., (0., 0.)) for i in range(150)]
        alarms = [row for row in rows if row['detector_status'] == 'alarm']
        self.assertTrue(alarms)
        self.assertLess((alarms[0]['sim_time_ns'] / 1e9) - 18., 10.)

    def test_threshold_and_invalid_time(self):
        self.assertAlmostEqual(CHI2_2_9999, 18.42068, places=4)
        detector = PlanarInnovationDetector()
        detector.step(0, 0., 0.)
        with self.assertRaises(ValueError):
            detector.step(0, 1., 0.)
        with self.assertRaises(ValueError):
            detector.step(1, math.nan, 0.)


if __name__ == '__main__':
    unittest.main()
