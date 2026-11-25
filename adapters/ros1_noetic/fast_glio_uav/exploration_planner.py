"""Sensor-only, fixed-height exploration on an observed-free occupancy grid.

The planner consumes registered LiDAR returns and synchronized LIO sensor poses.
It never loads a simulator world, truth pose, GPS or a precomputed route.
"""
import heapq
import math
import time
import copy
from collections import deque

import numpy as np


def advance_return_route(position, route, at_height, grid=None):
    """Keep a guidance target stable until reached; then advance safely."""
    route=list(route);reached=[]
    if grid is not None:
        grid._refresh()
        # Breadcrumbs are guidance, not obligations. New observations can make
        # an old point invalid. Discard invalid intermediate goals; the planner
        # must still verify a route to the next one. Never discard home itself.
        while len(route)>1 and (grid.cell(route[0]) in grid.blocked or grid.cell(route[0]) not in grid.free):
            route.pop(0)
    while route:
        threshold=1.8 if len(route)>1 else .65
        if not at_height or np.linalg.norm(np.asarray(position)[:2]-np.asarray(route[0])[:2])>=threshold:break
        reached.append(route.pop(0))
    if reached and grid is not None and route:
        route=route[grid.return_guidance_index(position,route):]
    return route,reached


class ExplorationPlanner:
    def __init__(self, height, resolution=1., clearance=2., max_cells=350000):
        self.height=float(height)
        self.resolution=float(resolution)
        self.clearance=float(clearance)
        self.max_cells=int(max_cells)
        self.free=set()
        self.occupied=set()
        self.visits={}
        self.updates=0
        self.plans=0
        self.blocked=set()
        self._dirty=True
        self.slab_half_height=2.
        radius=int(math.ceil(clearance/resolution))
        self.inflation=[(x,y) for x in range(-radius,radius+1) for y in range(-radius,radius+1)
                        if math.hypot(x,y)*resolution<=clearance+resolution*.71]

    def cell(self, point):
        return tuple(np.floor(np.asarray(point)[:2]/self.resolution).astype(int))

    def centre(self, cell):
        return np.array([(cell[0]+.5)*self.resolution,(cell[1]+.5)*self.resolution,self.height])

    def update(self, origin, points):
        """Clear only the part of a measured beam in the flight-height slab.

        No-return beams are absent from /cloud_registered and remain unknown.
        Obstacles accumulate conservatively; this baseline assumes a static map.
        """
        origin=np.asarray(origin,dtype=float)
        points=np.asarray(points,dtype=float).reshape(-1,3)
        points=points[np.isfinite(points).all(1)]
        if len(points)<100:return False
        # Control holds barometric height; LIO z may drift. Slice around the
        # synchronized sensor pose rather than a stale takeoff-frame altitude.
        self.height=float(origin[2])
        delta=points-origin
        distance=np.linalg.norm(delta[:,:2],axis=1)
        valid=(distance>.5)&(distance<100.)
        delta=delta[valid];distance=distance[valid]
        # Uniformly sample spatial bins instead of ring-major array strides.
        _,indices=np.unique(np.floor(points[valid]/.7).astype(int),axis=0,return_index=True)
        if len(indices)>5000:indices=indices[np.linspace(0,len(indices)-1,5000).astype(int)]
        delta=delta[indices];distance=distance[indices]
        dz=delta[:,2]
        lower=np.full(len(delta),-np.inf);upper=np.full(len(delta),np.inf)
        moving=np.abs(dz)>1.e-6
        a=(self.height-self.slab_half_height-origin[2])/np.where(moving,dz,1.)
        b=(self.height+self.slab_half_height-origin[2])/np.where(moving,dz,1.)
        lower[moving]=np.minimum(a,b)[moving];upper[moving]=np.maximum(a,b)[moving]
        lower[~moving & (abs(origin[2]-self.height)>self.slab_half_height)]=2.
        start=np.maximum(0.,lower)
        # Stop 0.8 m before an actual return, and limit insertion to 60 m.
        end=np.minimum.reduce([upper,60./np.maximum(distance,1.),1.-.8/np.maximum(distance,1.)])
        good=end>start
        if good.any():
            d=delta[good];lo=start[good];hi=end[good]
            steps=np.linspace(0.,1.,int(60/self.resolution*2)+1)
            t=lo[:,None]+(hi-lo)[:,None]*steps
            xy=origin[:2]+d[:,None,:2]*t[:,:,None]
            cells=np.unique(np.floor(xy.reshape(-1,2)/self.resolution).astype(int),axis=0)
            self.free.update(map(tuple,cells))
        hits=origin+delta
        band=np.abs(hits[:,2]-self.height)<=self.slab_half_height
        cells=np.unique(np.floor(hits[band,:2]/self.resolution).astype(int),axis=0)
        self.occupied.update(map(tuple,cells))
        self.free.add(self.cell(origin))
        if len(self.free)+len(self.occupied)>self.max_cells:
            raise ValueError('occupancy_capacity_reached')
        self.updates+=1;self._dirty=True
        return True

    def _refresh(self):
        if self._dirty:
            self.blocked={(x+dx,y+dy) for x,y in self.occupied for dx,dy in self.inflation}
            self._dirty=False

    def known_clear(self, start, end):
        self._refresh()
        start=np.asarray(start);end=np.asarray(end)
        count=max(2,int(np.linalg.norm(end[:2]-start[:2])/self.resolution*3)+2)
        cells={self.cell(p) for p in np.linspace(start,end,count)}
        return all(c in self.free and c not in self.blocked for c in cells)

    def _neighbours(self, cell):
        x,y=cell
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
            other=(x+dx,y+dy)
            if other not in self.free or other in self.blocked:continue
            if dx and dy and any(c not in self.free or c in self.blocked for c in [(x+dx,y),(x,y+dy)]):continue
            yield other,math.hypot(dx,dy)*self.resolution

    def snapshot(self):
        self._refresh()
        result=copy.copy(self)
        result.free=frozenset(self.free);result.occupied=frozenset(self.occupied)
        result.blocked=frozenset(self.blocked)
        return result

    def obstacle_distance(self, point):
        """Conservative XY distance to occupied cell boxes, not their centres."""
        if not self.occupied:return math.inf
        cells=np.asarray(list(self.occupied),dtype=float)*self.resolution
        delta=np.maximum(np.maximum(cells-np.asarray(point)[:2],np.asarray(point)[:2]-cells-self.resolution),0.)
        return float(np.linalg.norm(delta,axis=1).min())

    def recovery_clear(self, start, end):
        """Only retreat through observed free cells with nondecreasing clearance.

        This exception applies to inflation margins, never occupied or unknown
        cells. A physical minimum remains enforced along the whole segment.
        """
        start=np.asarray(start);end=np.asarray(end)
        if np.linalg.norm(end[:2]-start[:2])>6.:return False
        minimum=self.obstacle_distance(start)
        if minimum<.8:return False
        previous=minimum
        for point in np.linspace(start,end,max(2,int(np.linalg.norm(end[:2]-start[:2])*5)+2)):
            cell=self.cell(point);distance=self.obstacle_distance(point)
            if cell not in self.free or cell in self.occupied or distance<.8 or distance<previous-.025:return False
            previous=distance
        return True

    def recovery_plan(self, start, info):
        root=self.cell(start)
        if root in self.occupied or self.obstacle_distance(start)<.8:
            return dict(path=[],reason='unsafe_recovery_start',**info)
        queue=deque([(root,[np.asarray(start)])]);seen={root}
        while queue and len(seen)<500:
            cell,path=queue.popleft()
            if cell not in self.blocked:
                return dict(path=[p.tolist() for p in path],reason='recovering_clearance',**info)
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                other=(cell[0]+dx,cell[1]+dy);point=self.centre(other)
                if other in seen or np.linalg.norm(point[:2]-start[:2])>6.:continue
                if self.recovery_clear(path[-1],point):
                    seen.add(other);queue.append((other,path+[point]))
        return dict(path=[],reason='no_safe_retreat',**info)

    def steering_target(self, start, path, recovery=False, lookahead=3.):
        """Track one segment at a time; never shortcut across a path corner."""
        start=np.asarray(start);path=np.asarray(path)
        if len(path)<2:return None
        best=None
        for i in range(len(path)-1):
            delta=path[i+1,:2]-path[i,:2]
            t=np.clip(np.dot(start[:2]-path[i,:2],delta)/max(np.dot(delta,delta),1.e-9),0.,1.)
            point=path[i]+t*(path[i+1]-path[i])
            distance=np.linalg.norm(start[:2]-point[:2])
            if best is None or distance<best[0]:best=(distance,i,point)
        _,index,point=best
        delta=path[index+1]-point;delta[2]=0.
        target=point+delta*min(1.,lookahead/max(np.linalg.norm(delta),1.e-9));target[2]=start[2]
        check=self.recovery_clear if recovery else self.known_clear
        return target if check(start,target) else None

    def return_guidance_index(self, start, route, lookahead=8.):
        """Skip dense breadcrumbs only inside an observed safe direct corridor."""
        selected=0
        for index,point in enumerate(route[:12]):
            if np.linalg.norm(np.asarray(point)[:2]-np.asarray(start)[:2])>lookahead:break
            if not self.known_clear(start,point):break
            selected=index
        return selected

    def plan(self, start, goal, radius=45., budget=18000, goal_radius=0.):
        before=time.monotonic();self._refresh();self.plans+=1
        start=np.asarray(start);goal=np.asarray(goal);root=self.cell(start);destination=self.cell(goal)
        info=dict(free_cells=len(self.free),occupied_cells=len(self.occupied),updates=self.updates,plans=self.plans)
        if root in self.blocked:return self.recovery_plan(start,info)
        self.free.add(root)
        self.visits[root]=self.visits.get(root,0)+1
        targets={destination} if goal_radius<=0 else {
            c for c in self.free if c not in self.blocked and
            np.linalg.norm(self.centre(c)[:2]-goal[:2])<=goal_radius}
        # A sub-cell arrival radius may contain no grid centre. Preserve a
        # verified exact goal as a candidate instead of declaring it unreachable.
        if destination in self.free and destination not in self.blocked:targets.add(destination)
        if goal_radius<=0 and destination in self.blocked:return dict(path=[],reason='goal_occupied',**info)
        info['goal_radius_m']=goal_radius
        info['goal_centre_blocked']=destination in self.blocked
        # First try A* to the goal when it is observed. Otherwise expand the
        # reachable local free region and select a goal-biased frontier.
        penalties={}
        for (x,y),value in self.visits.items():
            if math.hypot(x-root[0],y-root[1])*self.resolution>radius+5:continue
            for dx in range(-3,4):
                for dy in range(-3,4):
                    weight=max(0.,1.-math.hypot(dx,dy)/4.)
                    key=(x+dx,y+dy);penalties[key]=penalties.get(key,0.)+value*weight
        direct=bool(targets) and (destination in self.free or goal_radius>0)
        queue=[(0.,0.,root)];cost={root:0.};parents={};expanded=set();frontiers=[];end=None
        while queue and len(expanded)<budget:
            _,g,cell=heapq.heappop(queue)
            if cell in expanded:continue
            expanded.add(cell)
            if cell in targets and cell not in self.blocked:end=cell;break
            unknown=sum((cell[0]+dx,cell[1]+dy) not in self.free and (cell[0]+dx,cell[1]+dy) not in self.blocked
                        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)])
            if unknown and g>=3.:
                position=self.centre(cell)
                revisit=penalties.get(cell,0.)
                score=np.linalg.norm(position[:2]-goal[:2])+.12*g+6.*revisit-.2*unknown
                frontiers.append((score,cell))
            for other,step in self._neighbours(cell):
                ng=g+step
                if not direct and ng>radius:continue
                if ng>=cost.get(other,math.inf):continue
                cost[other]=ng;parents[other]=cell
                heuristic=max(0.,math.hypot(other[0]-destination[0],other[1]-destination[1])*self.resolution-goal_radius) if direct else 0.
                heapq.heappush(queue,(ng+heuristic,ng,other))
        if end is None and frontiers:end=min(frontiers)[1]
        if end is None:return dict(path=[],reason='goal_region_unreachable' if goal_radius>0 else 'no_reachable_frontier',expanded=len(expanded),**info)
        cells=[end]
        while cells[-1]!=root:cells.append(parents[cells[-1]])
        raw=[start.copy()]+[self.centre(c) for c in reversed(cells[:-1])]
        if end==destination and self.known_clear(raw[-1],goal):raw.append(goal.copy())
        path=[raw[0]];index=0
        while index<len(raw)-1:
            furthest=index+1
            for j in range(index+2,len(raw)):
                if np.linalg.norm(raw[j]-raw[index])>8. or not self.known_clear(raw[index],raw[j]):break
                furthest=j
            path.append(raw[furthest]);index=furthest
        return dict(path=[p.tolist() for p in path],reason='goal_region_path' if goal_radius>0 and end in targets else 'goal_path' if end==destination else 'exploring_frontier',
                    expanded=len(expanded),planning_ms=(time.monotonic()-before)*1000,**info)
