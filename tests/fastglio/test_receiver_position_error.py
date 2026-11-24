"""Compile and sample the exact stochastic kernel used by the Gazebo plugin."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]


class ReceiverPositionErrorTest(unittest.TestCase):
    def test_generated_covariance_and_time_correlation_at_two_cadences(self):
        compiler=shutil.which('c++')
        if not compiler:self.skipTest('C++ compiler not available')
        with tempfile.TemporaryDirectory() as folder:
            binary=Path(folder)/'receiver-position-error'
            subprocess.run([compiler,'-std=c++17','-O2','-I',str(ROOT/'adapters/gazebo_classic'),
                            str(ROOT/'tests/fastglio/test_receiver_position_error.cpp'),'-o',str(binary)],check=True,capture_output=True)
            report=json.loads(subprocess.check_output([str(binary)],text=True))
        self.assertTrue(report['passed']);self.assertEqual(len(report['cadences']),2)


if __name__=='__main__':unittest.main()
