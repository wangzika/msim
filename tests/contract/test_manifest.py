import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[2] / 'tools/validation/create_manifest.py'


class ManifestTest(unittest.TestCase):
    def test_creation_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            subprocess.run(['git', 'init', '-q', str(root)], check=True)
            subprocess.run(['git', '-C', str(root), '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '--allow-empty', '-qm', 'test'], check=True)
            scenario = root / 'scenario.yaml'
            scenario.write_text('scenario_id: example\n')
            output = root / 'manifest.json'
            artifact = root / 'output.dat'
            artifact.write_bytes(b'example')
            cmd = [sys.executable, str(SCRIPT), '--repo', str(root), '--scenario', str(scenario), '--seed', '42', '--output', str(output), '--resolved-scenario', str(scenario), '--artifact', str(artifact), '--dependency-lock', str(scenario), '--image-digest', 'test@sha256:' + 'a' * 64]
            self.assertEqual(subprocess.run(cmd, capture_output=True).returncode, 0)
            data = json.loads(output.read_text())
            self.assertEqual(data['scenario']['sha256'], hashlib.sha256(scenario.read_bytes()).hexdigest())
            self.assertEqual(data['seed'], 42)
            self.assertEqual(data['resolved_scenario']['sha256'], data['scenario']['sha256'])
            self.assertEqual(data['artifacts'][0]['sha256'], hashlib.sha256(b'example').hexdigest())
            self.assertEqual(data['dependency_locks'][0]['sha256'], data['scenario']['sha256'])
            self.assertEqual(data['image_digests'], ['test@sha256:' + 'a' * 64])
            self.assertTrue(data['git']['dirty'])
            before = output.read_bytes()
            self.assertNotEqual(subprocess.run(cmd, capture_output=True).returncode, 0)
            self.assertEqual(output.read_bytes(), before)
            self.assertNotEqual(subprocess.run(cmd + ['--output', str(root / 'invalid.json'), '--image-digest', 'latest'], capture_output=True).returncode, 0)
            self.assertFalse((root / 'invalid.json').exists())
            self.assertNotEqual(subprocess.run(cmd + ['--output', str(root / 'other.json'), '--seed', '-1'], capture_output=True).returncode, 0)

    def test_missing_scenario_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run([sys.executable, str(SCRIPT), '--scenario', str(Path(tmp) / 'missing'), '--seed', '1', '--output', str(Path(tmp) / 'out')], capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((Path(tmp) / 'out').exists())


if __name__ == '__main__':
    unittest.main()
