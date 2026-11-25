import sys
from pathlib import Path
import unittest
import time
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'adapters/ros1_noetic/fast_glio_uav'))
from exploration_planner import ExplorationPlanner

class ExplorationRolloutTest(unittest.TestCase):
    def test_sensor_rays_explore_300m_around_wall_without_known_route(self):
        p=ExplorationPlanner(10);position=np.array([0.,0.,10.]);goal=np.array([0.,300.,10.])
        segments=[(-50,-30,-50,350),(50,-30,50,350),(-50,-30,50,-30),(-50,350,50,350),(-12,80,12,80),(-12,140,12,140),(-12,80,-12,140),(12,80,12,140)]
        a=np.linspace(-np.pi,np.pi,600,endpoint=False);rays=np.column_stack((np.cos(a),np.sin(a)));started=time.monotonic();path=[]
        for step in range(260):
         best=np.full(len(a),100.)
         for x,y,xx,yy in segments:
          edge=np.array([xx-x,yy-y]);offset=np.array([x,y])-position[:2]
          det=rays[:,0]*edge[1]-rays[:,1]*edge[0];valid=abs(det)>1.e-8
          d=(offset[0]*edge[1]-offset[1]*edge[0])/np.where(valid,det,1)
          t=(offset[0]*rays[:,1]-offset[1]*rays[:,0])/np.where(valid,det,1)
          valid&=(d>.1)&(t>=0)&(t<=1);best=np.minimum(best,np.where(valid,d,100.))
         valid=best<99.9;cloud=np.column_stack((position[:2]+rays[valid]*best[valid,None],np.full(valid.sum(),10.)))
         p.update(position,cloud);plan=p.plan(position,goal)
         if not plan['path']:raise RuntimeError((step,position,plan['reason']))
         q=np.asarray(plan['path']);target=q[1]
         delta=target-position;position+=delta*min(1.,2./max(np.linalg.norm(delta),1.e-9));path.append(position.copy())
         assert not (-13.5<position[0]<13.5 and 78.5<position[1]<141.5), position
         if np.linalg.norm(position-goal)<2:break
        else:raise RuntimeError(('not reached',position))
        self.assertLess(np.linalg.norm(position-goal),2.)
        self.assertGreater(max(abs(v[0]) for v in path),13.5)
