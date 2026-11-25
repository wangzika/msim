from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]

class CaptureSnapshotTest(unittest.TestCase):
    def test_running_driver_is_immutable_and_removed_after_exit(self):
        source=(ROOT/'tools/px4-uav-capture.sh').read_text().split('FAST_GLIO_ROOT=',1)[0]
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'driver.sh'
            original=source+'printf "%s\\n" "$MSIM_CAPTURE_SNAPSHOT"\nread -r release\nprintf "completed\\n"\n'
            path.write_text(original)
            env={k:v for k,v in os.environ.items() if not k.startswith('MSIM_CAPTURE_')}
            child=subprocess.Popen(['bash',str(path)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,env=env)
            try:
                snapshot=Path(child.stdout.readline().strip())
                self.assertNotEqual(snapshot,path);self.assertEqual(snapshot.read_text(),original)
                path.write_text("broken='")
                self.assertEqual(snapshot.read_text(),original)
                out,err=child.communicate('\n',timeout=5)
                self.assertEqual(child.returncode,0,err);self.assertEqual(out,'completed\n')
                self.assertFalse(snapshot.exists())
            finally:
                if child.poll() is None:child.kill();child.communicate()

if __name__=='__main__':unittest.main()
