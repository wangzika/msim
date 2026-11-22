from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       'adapters/ros1_noetic/fast_glio_uav'))
from sensor_sync import PastSampleBuffer, summarize_lidar_alignment


class SensorSyncTest(unittest.TestCase):
    def test_never_pairs_a_future_lidar_scan(self):
        scans = PastSampleBuffer()
        scans.add(1_000_000_000, 'earlier')
        scans.add(1_120_000_000, 'future')
        match = scans.match(1_100_000_000, 150_000_000)
        self.assertEqual(match['status'], 'matched')
        self.assertEqual(match['value'], 'earlier')
        self.assertEqual(match['age_ms'], 100.0)

    def test_stale_and_missing_scans_are_not_used(self):
        scans = PastSampleBuffer()
        scans.add(1_100_000_000, 'future')
        self.assertEqual(scans.match(1_000_000_000, 150_000_000)['status'],
                         'missing')
        scans.add(700_000_000, 'old')
        stale = scans.match(1_000_000_000, 150_000_000)
        self.assertEqual(stale['status'], 'stale')
        self.assertIsNone(stale['value'])
        self.assertEqual(stale['age_ms'], 300.0)

    def test_out_of_order_callbacks_choose_closest_past_stamp(self):
        scans = PastSampleBuffer()
        scans.add(1_050_000_000, 1)
        scans.add(1_000_000_000, 2)
        self.assertEqual(scans.match(1_080_000_000, 150_000_000)['value'], 1)

    def test_report_skips_startup_second(self):
        rows = [dict(sim_time_ns=0, lidar_sync_status='missing'),
                dict(sim_time_ns=1_000_000_000, lidar_sync_status='matched',
                     lidar_stamp_ns=960_000_000, lidar_age_ms=40.0,
                     lidar_point_count=12000),
                dict(sim_time_ns=1_200_000_000, lidar_sync_status='matched',
                     lidar_stamp_ns=1_140_000_000, lidar_age_ms=60.0,
                     lidar_point_count=12000)]
        report = summarize_lidar_alignment(rows)
        self.assertEqual(report['lidar_epoch_count'], 2)
        self.assertEqual(report['lidar_sync_fraction'], 1.0)
        self.assertEqual(report['lidar_max_age_ms'], 60.0)

    def test_report_recomputes_age_instead_of_trusting_status(self):
        rows = [dict(sim_time_ns=1_000_000_000,
                     lidar_sync_status='matched', lidar_stamp_ns=1_010_000_000,
                     lidar_age_ms=0.0, lidar_point_count=12000)]
        self.assertEqual(summarize_lidar_alignment(rows)['lidar_sync_fraction'], 0.0)


if __name__ == '__main__':
    unittest.main()
