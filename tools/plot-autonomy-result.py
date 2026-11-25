#!/usr/bin/env python3
from pathlib import Path
import sys,json,argparse
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'build/px4-pydeps'))
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
parser=argparse.ArgumentParser(description='Export autonomous return truth, LIO and GPS trajectories')
parser.add_argument('--run',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();run=args.run
nav=json.loads((run/'navigation-report.json').read_text())
truth=[];lio=[]
with rosbag.Bag(str(run/'sensors.bag')) as bag:
 for topic,msg,_ in bag.read_messages(topics=['/ground_truth/state','/Odometry']):
  p=msg.pose.pose.position
  (truth if topic=='/ground_truth/state' else lio).append([msg.header.stamp.to_sec(),p.x,p.y,p.z])
a=np.array(truth);b=np.array(lio);anchor=a[0,1:].copy();a[:,1:]-=anchor;b[:,1:]-=b[0,1:].copy()
rows=[json.loads(s) for s in (run/'injected-gps.jsonl').read_text().splitlines()]
g=np.array([[r['sim_time_ns']/1e9,r['gps_east_m'],r['gps_north_m'],r['gps_up_m']] for r in rows]);g[:,1:]-=np.array([rows[0]['east_m'],rows[0]['north_m'],rows[0]['up_m']])
ret=next(e['sim_time_s'] for e in nav['events'] if e['state']=='returning')
land=next(e['sim_time_s'] for e in nav['events'] if e['state']=='landing')
plt.rcParams.update({'font.family':'Noto Sans CJK SC','font.size':15})
fig=plt.figure(figsize=(13,5.8));ax=fig.add_subplot(121,projection='3d');p=fig.add_subplot(122)
ax.plot(*a[:,1:].T,':',color='#444444',lw=2.8,label='真实轨迹')
ax.plot(*b[:,1:].T,color='#158fb0',lw=1.8,label='LiDAR / IMU')
ax.plot(*g[:,1:].T,'--',color='#de7051',lw=1.8,label='受骗 GPS')
ax.set(xlabel='东 E / m',ylabel='北 N / m',zlabel='天 U / m',xticks=[0,5,10,15],yticks=[0,4,8],zticks=[0,2.5,5]);ax.tick_params(labelsize=12);ax.view_init(25,-50);ax.legend(loc='upper left',fontsize=12)
for low,high,label,color in [(a[0,0],ret,'出航','#158fb0'),(ret,land,'返航','#7e4da5'),(land,a[-1,0]+1,'降落','#777777')]:
 v=a[(a[:,0]>=low)&(a[:,0]<high)];p.plot(v[:,1],v[:,2],color=color,lw=2.5,label=label)
p.scatter([0],[0],color='#222222',s=80,marker='*',label='起飞点');p.scatter(a[-1,1],a[-1,2],facecolors='none',edgecolors='#7e4da5',s=100,label='落点')
p.set(xlabel='东 E / m',ylabel='北 N / m',xlim=(-.7,.7));p.grid(alpha=.25);p.legend(loc='upper right',fontsize=12)
fig.tight_layout();args.output.parent.mkdir(parents=True,exist_ok=True)
fig.savefig(args.output,dpi=180,bbox_inches='tight');print(args.output)
