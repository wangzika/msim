#!/usr/bin/env python3
"""Export the recorded manual-flight truth and LiDAR/IMU trajectory."""
import argparse
from bisect import bisect_left
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True)
    run=ap.parse_args().run
    tracks={'/ground_truth/state':[],'/Odometry':[]}
    with rosbag.Bag(str(run/'sensors.bag')) as bag:
        for topic,msg,_ in bag.read_messages(topics=list(tracks)):
            p=msg.pose.pose.position
            tracks[topic].append([msg.header.stamp.to_sec(),p.x,p.y,p.z])
    if any(len(v)<2 for v in tracks.values()):
        print('轨迹消息不足，原始录包和地图已保留。');return
    truth=np.array(tracks['/ground_truth/state']);lio=np.array(tracks['/Odometry'])
    at=bisect_left(truth[:,0],lio[0,0]);at=min(at,len(truth)-1)
    near=min([at,max(0,at-1)],key=lambda i:abs(truth[i,0]-lio[0,0]))
    anchor=truth[near,1:].copy();truth[:,1:]-=anchor;lio[:,1:]-=lio[0,1:].copy()
    plt.rcParams.update({'font.family':'Noto Sans CJK SC','font.size':15})
    fig=plt.figure(figsize=(13,6),facecolor='white')
    ax3=fig.add_subplot(121,projection='3d');ax2=fig.add_subplot(122)
    for rows,label,color,style in [(truth,'真实轨迹','#555555',':'),(lio,'LiDAR / IMU','#167DB1','-')]:
        ax3.plot(*rows[:,1:].T,label=label,color=color,linestyle=style,linewidth=2)
        ax2.plot(rows[:,1],rows[:,2],label=label,color=color,linestyle=style,linewidth=2)
    ax3.set(xlabel='东 E / m',ylabel='北 N / m',zlabel='天 U / m')
    ax3.view_init(elev=25,azim=-55)
    ax2.set(xlabel='东 E / m',ylabel='北 N / m');ax2.set_aspect('equal',adjustable='datalim')
    ax2.grid(alpha=.25);ax2.legend()
    fig.tight_layout();output=run/'teleop-trajectory.png';fig.savefig(output,dpi=200,bbox_inches='tight');plt.close(fig)
    print(output)


if __name__=='__main__':main()
