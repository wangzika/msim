#!/usr/bin/env python3
"""Replace simulated IMU attitude with gyro integration for a LIO-SAM ablation.

The initial quaternion is retained; every later quaternion is integrated only
from measured angular velocity and message stamps. Acceleration, gyro, clouds,
and truth are unchanged.
"""
import argparse
from pathlib import Path

import numpy as np
import rosbag
from scipy.spatial.transform import Rotation


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True, type=Path)
    args = ap.parse_args()
    source = args.run / 'comparisons/annotated-sensors.bag'
    target = args.run / 'comparisons/gyro-orientation-sensors.bag'
    attitude = None
    last_time = None
    last_omega = None
    with rosbag.Bag(str(source)) as reader, \
            rosbag.Bag(str(target), 'w', compression='lz4') as writer:
        for topic, msg, stamp in reader.read_messages():
            if topic == '/imu/data':
                now = msg.header.stamp.to_sec()
                omega = np.array([msg.angular_velocity.x,
                                  msg.angular_velocity.y,
                                  msg.angular_velocity.z])
                if attitude is None:
                    q = msg.orientation
                    attitude = Rotation.from_quat([q.x, q.y, q.z, q.w])
                else:
                    dt = now - last_time
                    if dt < 0:
                        raise ValueError('IMU stamps are not monotonic')
                    attitude = attitude * Rotation.from_rotvec(
                        0.5 * (last_omega + omega) * dt)
                q = attitude.as_quat()
                msg.orientation.x, msg.orientation.y = q[0], q[1]
                msg.orientation.z, msg.orientation.w = q[2], q[3]
                last_time, last_omega = now, omega
            writer.write(topic, msg, stamp)
    print(target)


if __name__ == '__main__':
    main()
