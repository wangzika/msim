#!/usr/bin/env python3
"""Export a goal exploration result from evaluation-only truth and recorded plans."""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Circle
from ksql_mesh import load_ksql


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args();run=args.run;nav=json.loads((run/'navigation-report.json').read_text())
    goal=np.asarray(nav['goal_relative_enu_m']);truth=[];plans=[];last=-np.inf
    with rosbag.Bag(str(run/'sensors.bag')) as bag:
        for topic,msg,_ in bag.read_messages(topics=['/ground_truth/state','/msim/planned_path']):
            if topic=='/ground_truth/state':
                p=msg.pose.pose.position;truth.append([msg.header.stamp.to_sec(),p.x,p.y,p.z])
            elif msg.header.stamp.to_sec()-last>25 and msg.poses:
                last=msg.header.stamp.to_sec();plans.append([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses])
    rows=np.asarray(truth);home=rows[0,1:].copy();rows[:,1:]-=home
    plt.rcParams.update({'font.family':'Noto Sans CJK SC','font.size':15})
    fig=plt.figure(figsize=(12.5,7));grid=fig.add_gridspec(1,2,width_ratios=[.8,1.2])
    ax=fig.add_subplot(grid[0,0]);right=fig.add_subplot(grid[0,1])
    if json.loads((run/'capture-settings.json').read_text()).get('scene_key')=='ksql_airport':
        triangles=load_ksql(Path(__file__).resolve().parents[1]/'build/scenes/ksql-airport')['Buildings']-home
        normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);norm=np.linalg.norm(normal,axis=1)
        roof=np.abs(normal[:,2])/np.maximum(norm,1.e-9)>.7
        lo=rows[:,1:3].min(0)-20;hi=rows[:,1:3].max(0)+20
        roof&=np.all(triangles[:,:,:2].max(1)>=lo,axis=1)&np.all(triangles[:,:,:2].min(1)<=hi,axis=1)
        ax.add_collection(PolyCollection(triangles[roof,:,:2],facecolors='#cdd3d9',edgecolors='none',alpha=.7))
    outbound=next((e['sim_time_s'] for e in nav['events'] if e['state']=='outbound'),rows[0,0])
    returning=next((e['sim_time_s'] for e in nav['events'] if e['state']=='returning'),rows[-1,0])
    for low,high,label,color in [(outbound,returning,'自主探索','#168ca6'),(returning,rows[-1,0]+1,'自主返航','#9054b5')]:
        p=rows[(rows[:,0]>=low)&(rows[:,0]<high)]
        ax.plot(p[:,1],p[:,2],lw=3.5 if label=='自主探索' else 1.8,linestyle='-' if label=='自主探索' else '--',color=color,label=label)
    for p in plans:
        p=np.asarray(p)-np.asarray(nav['home_lio']);ax.plot(p[:,0],p[:,1],color='#ad8bbf',alpha=.4,lw=.7)
    ax.scatter([0],[0],marker='*',s=140,color='#222222',label='起飞点',zorder=5)
    ax.scatter([goal[0]],[goal[1]],marker='X',s=100,color='#d36c45',label='目标中心',zorder=5)
    ax.add_patch(Circle(goal[:2],float(nav.get('goal_radius_m',2)),fill=False,color='#d36c45',linestyle='--',label='目标允许区域'))
    ax.set(xlabel='东 E / m',ylabel='北 N / m',xlim=(rows[:,1].min()-20,rows[:,1].max()+50),ylim=(-15,max(goal[1],rows[:,2].max())+15))
    ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2);ax.legend(fontsize=11,ncol=2,loc='lower left',bbox_to_anchor=(0,1.02))
    seconds=rows[:,0]-rows[0,0]
    right.plot(seconds,np.linalg.norm(rows[:,1:3]-goal[:2],axis=1),label='距目标点',color='#168ca6',lw=2)
    right.plot(seconds,np.linalg.norm(rows[:,1:3],axis=1),label='距起飞点',color='#9054b5',lw=2)
    if nav.get('goal_reached_sim_time_s') is not None:
        right.axvline(nav['goal_reached_sim_time_s']-rows[0,0],color='#555555',linestyle=':',label='到达并返航')
    right.set(xlabel='任务时间 / s',ylabel='真实水平距离 / m');right.grid(alpha=.2);right.legend(fontsize=12)
    fig.tight_layout();out=args.output or run/'goal-mission-result.png';out.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(out,dpi=180,bbox_inches='tight');plt.close(fig);print(out)

if __name__=='__main__':main()
