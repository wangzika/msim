#!/usr/bin/env python3
"""Publish a reproducible noisy IMU from Gazebo's ideal IMU measurement."""

import rospy
import json
from pathlib import Path
from sensor_msgs.msg import Imu

from imu_noise_model import ImuConfig, ImuNoiseModel


def diagonal(variance):
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def main():
    rospy.init_node('msim_imu_model')
    # JSON scientific notation such as 2e-05 is a string in ROS's YAML 1.1
    # loader. Read the JSON snapshot directly to retain numeric types.
    raw = (json.loads(Path(rospy.get_param('~profile_path')).read_text())['imu_ros']
           if rospy.has_param('~profile_path') else rospy.get_param('~imu_ros'))
    config = ImuConfig.from_dict(raw)
    model = ImuNoiseModel(config)
    publisher = rospy.Publisher('/imu/data', Imu, queue_size=200)

    def on_ideal(msg):
        nonlocal model
        stamp_s = msg.header.stamp.to_sec()
        if model.last_stamp_s is not None and stamp_s < model.last_stamp_s:
            rospy.logwarn('simulation clock moved backwards; resetting IMU bias state')
            model = ImuNoiseModel(config)
        elif model.last_stamp_s is not None and stamp_s == model.last_stamp_s:
            return
        try:
            result = model.sample(
                stamp_s,
                (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
                (msg.linear_acceleration.x, msg.linear_acceleration.y,
                 msg.linear_acceleration.z),
                (msg.orientation.x, msg.orientation.y, msg.orientation.z,
                 msg.orientation.w))
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, 'invalid ideal IMU measurement: %s', exc)
            return
        output = Imu()
        output.header = msg.header
        output.orientation.x, output.orientation.y, output.orientation.z, \
            output.orientation.w = result['orientation']
        output.angular_velocity.x, output.angular_velocity.y, \
            output.angular_velocity.z = result['angular_velocity']
        output.linear_acceleration.x, output.linear_acceleration.y, \
            output.linear_acceleration.z = result['linear_acceleration']
        output.orientation_covariance = diagonal(result['orientation_variance'])
        output.angular_velocity_covariance = diagonal(result['gyro_variance'])
        output.linear_acceleration_covariance = diagonal(result['accel_variance'])
        publisher.publish(output)

    rospy.Subscriber('/imu/ideal', Imu, on_ideal, queue_size=200)
    rospy.loginfo('msim IMU model: independent accel/gyro noise, correlated bias, seed %d',
                  config.seed)
    rospy.spin()


if __name__ == '__main__':
    main()
