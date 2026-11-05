#!/usr/bin/env python3
"""Publish Gazebo truth as a path translated to FAST_LIO's initial map origin."""

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('msim_px4_truth_path')
    publisher = rospy.Publisher('/ground_truth/aligned_path', Path,
                                queue_size=1, latch=True)
    path = Path()
    path.header.frame_id = 'camera_init'
    origin = None
    count = 0

    def on_truth(msg):
        nonlocal origin, count
        count += 1
        p = msg.pose.pose.position
        if origin is None:
            origin = (p.x, p.y, p.z)
        if count % 10:
            return
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'camera_init'
        pose.pose.position.x = p.x - origin[0]
        pose.pose.position.y = p.y - origin[1]
        pose.pose.position.z = p.z - origin[2]
        pose.pose.orientation = msg.pose.pose.orientation
        path.header.stamp = msg.header.stamp
        path.poses.append(pose)
        publisher.publish(path)

    rospy.Subscriber('/ground_truth/state', Odometry, on_truth, queue_size=100)
    rospy.spin()


if __name__ == '__main__':
    main()
