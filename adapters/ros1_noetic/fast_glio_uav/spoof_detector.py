"""ROS adapter compatibility import for the portable innovation detector."""
from pathlib import Path
import sys

# ROS launches this adapter directly without installing the optional core wheel.
sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from msim.detector import CHI2_2_9999,PlanarInnovationDetector,horizontal_acceleration_values


def horizontal_acceleration(imu):
    q=imu.orientation;v=imu.linear_acceleration
    return horizontal_acceleration_values((v.x,v.y,v.z),(q.x,q.y,q.z,q.w))
