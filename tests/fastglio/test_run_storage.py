import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'adapters/ros1_noetic/fast_glio_uav'))
from run_storage import archive_runtime,recording_root,runtime_directory,compatibility_link


class RunStorageTest(unittest.TestCase):
    def test_compatibility_links_preserve_old_paths_and_explicit_other_storage(self):
        with tempfile.TemporaryDirectory() as folder:
            project=Path(folder)/'project';external=Path(folder)/'external';(project/'config').mkdir(parents=True)
            (project/'config/run-storage.json').write_text(json.dumps(dict(run_root=str(external),mount_point='/external')))
            with patch.dict(os.environ,{},clear=True),patch('run_storage.os.path.ismount',return_value=True):
                compatibility_link(project,external/'case')
                self.assertTrue((project/'runs/case').is_symlink())
                self.assertEqual((project/'runs/case').resolve(),external/'case')
            with patch.dict(os.environ,{},clear=True),patch('run_storage.os.path.ismount',return_value=False):
                compatibility_link(project,Path(folder)/'explicit-tmp-case')
                with self.assertRaises(RuntimeError):compatibility_link(project,external/'another-case')

    def test_external_config_requires_mounted_disk_and_override_is_explicit(self):
        with tempfile.TemporaryDirectory() as folder:
            project=Path(folder);(project/'config').mkdir()
            (project/'config/run-storage.json').write_text(json.dumps(dict(run_root='/external/runs',mount_point='/external')))
            with patch.dict(os.environ,{},clear=True),patch('run_storage.os.path.ismount',return_value=False):
                with self.assertRaisesRegex(RuntimeError,'未挂载'):recording_root(project)
            with patch.dict(os.environ,dict(MSIM_RUN_ROOT=str(project/'custom'))):
                self.assertEqual(recording_root(project),project/'custom')

    def test_posix_runtime_is_used_when_symlinks_are_not_supported(self):
        with tempfile.TemporaryDirectory() as folder:
            run=Path(folder)/'recording'
            self.assertEqual(runtime_directory(run),run)
            self.assertFalse(run.exists(),'The capture directory must still be created by the recorder')
            with patch.object(Path,'symlink_to',side_effect=OSError('exfat')):
                runtime=runtime_directory(run)
            self.assertEqual(runtime.parent,Path(tempfile.gettempdir()))
            self.assertTrue(runtime.name.startswith('msim-runtime-'));shutil.rmtree(runtime)

    def test_runtime_archive_preserves_logs_and_link_metadata_before_cleanup(self):
        runtime=Path(tempfile.mkdtemp(prefix='msim-runtime-test-'))
        with tempfile.TemporaryDirectory() as folder:
            run=Path(folder)/'run';(runtime/'ros-home/log').mkdir(parents=True)
            (runtime/'ros-home/log/flight.ulg').write_bytes(b'flight-log\0data')
            (runtime/'ros-home/etc').symlink_to('/external/px4/etc')
            result=archive_runtime(runtime,run,cleanup=True)
            self.assertFalse(runtime.exists())
            self.assertEqual((run/'ros-home/log/flight.ulg').read_bytes(),b'flight-log\0data')
            self.assertEqual(result['symlinks'],[dict(path='ros-home/etc',target='/external/px4/etc')])
            self.assertTrue(result['files'][0]['sha256'])
