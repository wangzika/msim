from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'adapters/ros1_noetic/fast_glio_uav'))
from capture_health import raise_if_simulation_crashed,raise_recorder_exit,require_recording_space
from unittest.mock import patch
from types import SimpleNamespace


class CaptureHealthTest(unittest.TestCase):
    def test_disk_full_is_reported_as_recorder_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'record.log';path.write_text('Less than 1G of space free on disk. Disabling recording.\n')
            with self.assertRaisesRegex(RuntimeError,'磁盘空间不足'):raise_recorder_exit(path,1)
            with patch('capture_health.shutil.disk_usage',return_value=SimpleNamespace(free=100)):
                with self.assertRaisesRegex(RuntimeError,'录包磁盘空间不足'):require_recording_space(Path(tmp))

    def test_crash_is_reported_instead_of_waiting_for_downstream_gnss(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'launch.log'
            raise_if_simulation_crashed(path)
            path.write_text('Ready for takeoff!\nSegmentation fault (core dumped)\n')
            with self.assertRaisesRegex(RuntimeError,'simulation crashed.*GNSS'):
                raise_if_simulation_crashed(path)
            path.write_text('x'*100000+'\n[gazebo-3] process has died [pid 123, exit code -11]\n')
            with self.assertRaises(RuntimeError):raise_if_simulation_crashed(path)

    def test_nonfatal_warnings_are_not_crashes(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'launch.log';path.write_text('WARNING: Package name\nmissing xyzOffset, set to default\nReady for takeoff!\n')
            raise_if_simulation_crashed(path)
