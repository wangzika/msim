#!/usr/bin/env python3
"""Publish aligned truth for RViz and world LIO for the Gazebo overlay only."""
from collections import deque
import copy
import json
import os
from pathlib import Path
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from truth_comparison import TruthComparison


def main():
    rospy.init_node('msim_truth_comparison')
    comparison=TruthComparison();lock=threading.RLock();poses=deque(maxlen=5000)
    report_file=rospy.get_param('~report_file',os.environ.get('MSIM_TRUTH_COMPARISON_REPORT',''))
    aligned_path=rospy.Publisher('/ground_truth/aligned_path',RosPath,queue_size=1,latch=True)
    aligned_odom=rospy.Publisher('/ground_truth/aligned_odom',Odometry,queue_size=5)
    world_lio=rospy.Publisher('/msim/lio_world_odom',Odometry,queue_size=5)
    def save_report():
        if report_file:
            with lock:
                target=Path(report_file);temporary=target.with_suffix('.json.tmp')
                temporary.write_text(json.dumps(comparison.report(),indent=2)+'\n');temporary.replace(target)
    rospy.on_shutdown(save_report)
    def truth(msg):
        if msg.header.frame_id not in ('world','map','enu'):
            rospy.logwarn_throttle(5,'Unsupported ground-truth frame for display: %s',msg.header.frame_id);return
        p=msg.pose.pose.position
        with lock:
            if comparison.truth and msg.header.stamp.to_nsec()<comparison.truth[-1][0]:poses.clear()
            q=msg.pose.pose.orientation
            comparison.add_truth(msg.header.stamp.to_nsec(),(p.x,p.y,p.z),(q.x,q.y,q.z,q.w))
    def lio(msg):
        p=msg.pose.pose.position
        if msg.header.frame_id!='camera_init':
            rospy.logwarn_throttle(5,'Truth comparison requires camera_init with ENU-aligned axes; refusing to rename another frame');return
        with lock:
            row=comparison.match(msg.header.stamp.to_nsec(),(p.x,p.y,p.z))
            if row is None:return
            out=Odometry();out.header=copy.deepcopy(msg.header);out.child_frame_id='truth_body'
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=row['truth_aligned']
            q=out.pose.pose.orientation
            q.x,q.y,q.z,q.w=row['truth_orientation']
            aligned_odom.publish(out)
            point=PoseStamped();point.header=copy.deepcopy(out.header);point.pose=copy.deepcopy(out.pose.pose)
            poses.append(point);path=RosPath();path.header=copy.deepcopy(out.header);path.poses=list(poses);aligned_path.publish(path)
            estimate=copy.deepcopy(msg);estimate.header.frame_id='world'
            estimate.pose.pose.position.x,estimate.pose.pose.position.y,estimate.pose.pose.position.z=row['lio_world']
            world_lio.publish(estimate)
            if comparison.matches%10==1:save_report()
    rospy.Subscriber('/ground_truth/state',Odometry,truth,queue_size=200)
    rospy.Subscriber('/Odometry',Odometry,lio,queue_size=20)
    rospy.spin()


if __name__=='__main__':main()
