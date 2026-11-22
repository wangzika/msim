#!/usr/bin/env python3
"""Adapt instantaneous Gazebo clouds for FAST-LIO's Velodyne input."""
import importlib.util
from collections import deque
import threading
import numpy as np
from pathlib import Path
import rospy
from sensor_msgs.msg import PointCloud2
from livox_ros_driver.msg import CustomMsg

root=Path(__file__).resolve().parents[4]
spec=importlib.util.spec_from_file_location('msim_lidar_adapter',root/'tools/annotate-gazebo-lidar.py')
adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)


def main():
    rospy.init_node('msim_lidar_format')
    publisher=rospy.Publisher(adapter.OUTPUT_TOPIC,PointCloud2,queue_size=3)
    livox_publisher=rospy.Publisher('/msim/livox_lidar_for_fusion',CustomMsg,queue_size=3)
    delay=float(rospy.get_param('~processing_delay_s',0.0))
    if not np.isfinite(delay) or not 0<=delay<=2:
        raise ValueError('processing_delay_s must be between 0 and 2 seconds')
    pending={'cloud':deque(maxlen=64),'livox':deque(maxlen=64)}
    lock=threading.Lock()
    def enqueue(message,end,kind,pub):
        if delay==0:
            pub.publish(message)
            return
        with lock:
            q=pending[kind]
            if q and message.header.stamp<q[-1][1].header.stamp:
                q.clear()
            q.append((end+delay,message,pub))
    def callback(message):
        message=adapter.annotate(message)
        field=next(f for f in message.fields if f.name=='time')
        values=np.ndarray((message.width*message.height,),dtype='<f4',buffer=message.data,
                          offset=field.offset,strides=(message.point_step,)) if message.width else np.array([0.])
        end=message.header.stamp.to_sec()+float(values.max())
        enqueue(message,end,'cloud',publisher)
    def livox_callback(message):
        end=message.header.stamp.to_sec()+max((p.offset_time for p in message.points),default=0)*1e-9
        enqueue(message,end,'livox',livox_publisher)
    def flush(_):
        ready=[]
        with lock:
            for q in pending.values():
                while q and q[0][0]<=rospy.get_time():
                    _,msg,pub=q.popleft();ready.append((msg,pub))
        for msg,pub in ready:
            pub.publish(msg)
    timer=rospy.Timer(rospy.Duration(.01),flush,reset=True)
    rospy.Subscriber('/velodyne_points',PointCloud2,callback,queue_size=3,
                     buff_size=8*1024*1024,tcp_nodelay=True)
    rospy.Subscriber('/livox/lidar',CustomMsg,livox_callback,queue_size=3,
                     buff_size=8*1024*1024,tcp_nodelay=True)
    rospy.loginfo('LiDAR adapter: preserve native ring/time; legacy XYZI snapshots get uniform 1 us times')
    rospy.loginfo('Estimator LiDAR processing delay %.3f s; measurement timestamps preserved',delay)
    rospy.spin()


if __name__=='__main__':main()
