import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[2] / 'apps/orchestrator/recorded_run.py'
WORKER = '''import argparse
from pathlib import Path
p = argparse.ArgumentParser()
p.add_argument('--scenario'); p.add_argument('--seed'); p.add_argument('--output')
a = p.parse_args()
Path(a.output, 'result.txt').write_text(a.seed + ':' + Path(a.scenario).read_text())
'''


class RecordedRunTest(unittest.TestCase):
    def test_success_and_failure_are_recorded_without_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            subprocess.run(['git', 'init', '-q', str(root)], check=True)
            subprocess.run(['git', '-C', str(root), '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '--allow-empty', '-qm', 'test'], check=True)
            scenario = root / 'scenario.yaml'
            scenario.write_text('example\n')
            worker = root / 'worker.py'
            worker.write_text(WORKER)
            run = root / 'run'
            base = [sys.executable, str(SCRIPT), '--repo', str(root), '--scenario', str(scenario), '--seed', '7', '--output', str(run), '--lock', str(scenario), '--image', 'test@sha256:' + 'a' * 64, '--']
            result = subprocess.run(base + [sys.executable, str(worker)], capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = json.loads((run / 'manifest.json').read_text())
            self.assertEqual(manifest['exit_code'], 0)
            self.assertEqual(manifest['seed'], 7)
            self.assertEqual((run / manifest['dependency_locks'][0]['captured_path']).read_bytes(), scenario.read_bytes())
            self.assertEqual(manifest['scenario']['sha256'], hashlib.sha256(scenario.read_bytes()).hexdigest())
            self.assertEqual((run / 'artifacts/result.txt').read_text(), '7:example\n')
            self.assertIn('artifacts/result.txt', [item['path'] for item in manifest['files']])
            original = (run / 'manifest.json').read_bytes()
            self.assertNotEqual(subprocess.run(base + [sys.executable, str(worker)], capture_output=True).returncode, 0)
            self.assertEqual((run / 'manifest.json').read_bytes(), original)
            failed = root / 'failed'
            failure = [sys.executable, str(SCRIPT), '--repo', str(root), '--scenario', str(scenario), '--seed', '7', '--output', str(failed), '--', '/no/such/executable']
            self.assertNotEqual(subprocess.run(failure, capture_output=True).returncode, 0)
            self.assertEqual(json.loads((failed / 'manifest.json').read_text())['exit_code'], 127)


if __name__ == '__main__':
    unittest.main()
