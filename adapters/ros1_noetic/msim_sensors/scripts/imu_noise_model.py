"""ROS compatibility import; the shared IMU model has no ROS dependency."""
from pathlib import Path
import sys

root = str(Path(__file__).resolve().parents[4])
if root not in sys.path:
    sys.path.insert(0, root)
from msim.imu_model import ImuConfig, ImuNoiseModel
