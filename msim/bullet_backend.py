"""Optional Bullet rigid-body backend. No ROS, PX4 or estimator imports.

Iris parameters are sourced from the existing SDF. The rigid aggregate and
contact solver differ from Gazebo; this backend is experimental, not equivalent.
"""
import json
import math
from importlib.resources import files
import numpy as np
from .physics_scene import load_collada


def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class BulletBackend:
    name='bullet'
    def __init__(self,scene,spawn=(0,-20,.08),step_s=.002,vehicle=None):
        try:import pybullet as p
        except ImportError as exc:raise ValueError('Install optional physics dependency: python -m pip install ".[physics]"') from exc
        if not .0001<=step_s<=.01 or not math.isfinite(step_s):raise ValueError('Physics step must be 0.0001..0.01 s')
        self.p=p;self.client=p.connect(p.DIRECT);self.dt=step_s;self.tick=0
        self.cfg=vehicle or json.loads(files('msim').joinpath('data/iris.json').read_text(encoding='utf-8'))
        self.gravity=np.asarray(scene['gravity']);self.scene=scene;self.objects=[]
        self.mass=sum(link['mass_kg'] for link in self.cfg['links'])
        # Diagonal aggregate about the base origin. Full articulated propeller
        # gyroscopic and inertia cross terms are intentionally not claimed.
        self.inertia=np.zeros(3)
        for link in self.cfg['links']:
            pos=np.array(link['position_m']);self.inertia+=np.array(link['inertia_diagonal'])+link['mass_kg']*(pos@pos-pos*pos)
        self.rotors=self.cfg['rotors'];self.omega=np.zeros(4)
        self.positions=np.array([r['position_m'] for r in self.rotors])
        self.motor_k=np.array([r['motorConstant'] for r in self.rotors])
        # Reaction torque opposes propeller spin, same sign as Gazebo motor model.
        self.yaw_k=-np.array([r['spin']*r['momentConstant'] for r in self.rotors])
        self.mixer=np.vstack([np.ones(4),self.positions[:,1],-self.positions[:,0],self.yaw_k])
        try:
            p.setGravity(*self.gravity,physicsClientId=self.client)
            p.setTimeStep(step_s,physicsClientId=self.client)
            p.setPhysicsEngineParameter(numSolverIterations=50,deterministicOverlappingPairs=1,physicsClientId=self.client)
            for item in scene['objects']:self._object(item)
            shape=p.createCollisionShape(p.GEOM_BOX,halfExtents=(np.array(self.cfg['collision_box_m'])/2).tolist(),physicsClientId=self.client)
            self.vehicle=p.createMultiBody(baseMass=self.mass,baseCollisionShapeIndex=shape,basePosition=spawn,physicsClientId=self.client)
            p.changeDynamics(self.vehicle,-1,localInertiaDiagonal=self.inertia.tolist(),linearDamping=0,angularDamping=0,
                             lateralFriction=.8,restitution=0,physicsClientId=self.client)
            self.acceleration=np.zeros(3);self.angular_acceleration=np.zeros(3)
        except BaseException:
            self.close();raise

    def _object(self,item):
        p=self.p;c=self.client;kind=item['kind'];matrix=np.array(item['matrix'])
        r=matrix[:3,:3];angles=[math.atan2(r[2,1],r[2,2]),math.atan2(-r[2,0],math.hypot(r[0,0],r[1,0])),math.atan2(r[1,0],r[0,0])]
        options={}
        if kind=='box':shape=p.GEOM_BOX;options['halfExtents']=(np.array(item['size'])/2).tolist()
        elif kind=='sphere':shape=p.GEOM_SPHERE;options['radius']=item['radius']
        elif kind=='cylinder':shape=p.GEOM_CYLINDER;options.update(radius=item['radius'],height=item['length'])
        elif kind=='plane':shape=p.GEOM_PLANE;options['planeNormal']=item['normal']
        elif kind=='mesh':
            shape=p.GEOM_MESH
            if item['path'].lower().endswith('.dae'):
                triangles=load_collada(item['path'])*np.asarray(item['scale'])
                lo=triangles.min((0,1));hi=triangles.max((0,1))
                display=matrix.copy();display[:3,3]+=matrix[:3,:3]@((hi+lo)/2)
                item['bounds_size']=(hi-lo).tolist();item['display_matrix']=display.tolist()
                # Share identical vertices without removing any triangles. This
                # also avoids Bullet's command-buffer limit on large DAE lists.
                vertices,indices=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
                options.update(vertices=vertices.tolist(),indices=indices.tolist())
            elif item['path'].lower().endswith(('.obj','.stl')):
                options.update(fileName=item['path'],meshScale=item['scale'])
            else:raise ValueError('Unsupported collision mesh extension: '+item['path'])
            options['flags']=p.GEOM_FORCE_CONCAVE_TRIMESH
        else:raise ValueError('Unsupported geometry '+kind)
        collision=p.createCollisionShape(shape,**options,physicsClientId=c)
        if collision<0:raise ValueError('Collision mesh failed to load: '+item['name'])
        body=p.createMultiBody(baseMass=0,baseCollisionShapeIndex=collision,basePosition=matrix[:3,3].tolist(),
                               baseOrientation=p.getQuaternionFromEuler(angles),physicsClientId=c)
        self.objects.append(body)

    def state(self):
        pos,q=self.p.getBasePositionAndOrientation(self.vehicle,physicsClientId=self.client)
        velocity,omega=self.p.getBaseVelocity(self.vehicle,physicsClientId=self.client)
        return dict(stamp_ns=round(self.tick*self.dt*1e9),position=np.array(pos),orientation=np.array(q),
                    velocity=np.array(velocity),angular_velocity=rotation(q).T@np.array(omega),
                    acceleration=self.acceleration.copy(),angular_acceleration=self.angular_acceleration.copy())

    def step(self,motor_radps):
        command=np.asarray(motor_radps,dtype=float)
        if command.shape!=(4,) or not np.isfinite(command).all() or np.any(command<0):raise ValueError('Expected four nonnegative motor speeds')
        before=self.state();R=rotation(before['orientation']);v=R.T@before['velocity'];body_w=before['angular_velocity']
        torque=np.zeros(3)
        for i,r in enumerate(self.rotors):
            target=min(command[i],r['maxRotVelocity']);tau=r['timeConstantUp'] if target>self.omega[i] else r['timeConstantDown']
            self.omega[i]=target+(self.omega[i]-target)*math.exp(-self.dt/tau)
            force=np.array([0.,0.,self.motor_k[i]*self.omega[i]**2])
            perpendicular=v+np.cross(body_w,self.positions[i]);perpendicular[2]=0
            force-=abs(self.omega[i])*r['rotorDragCoefficient']*perpendicular
            self.p.applyExternalForce(self.vehicle,-1,force.tolist(),self.positions[i].tolist(),self.p.LINK_FRAME,physicsClientId=self.client)
            torque[2]+=self.yaw_k[i]*self.motor_k[i]*self.omega[i]**2
            torque-=abs(self.omega[i])*r['rollingMomentCoefficient']*perpendicular
        self.p.applyExternalTorque(self.vehicle,-1,torque.tolist(),self.p.LINK_FRAME,physicsClientId=self.client)
        self.p.stepSimulation(physicsClientId=self.client);self.tick+=1
        after=self.state();self.acceleration=(after['velocity']-before['velocity'])/self.dt
        # This derivative is only used for the IMU lever acceleration term.
        self.angular_acceleration=(after['angular_velocity']-before['angular_velocity'])/self.dt
        return self.state()

    def contacts(self):
        return [dict(other=int(row[2] if row[1]==self.vehicle else row[1]),normal_force_n=float(row[9]))
                for row in self.p.getContactPoints(bodyA=self.vehicle,physicsClientId=self.client)]

    def rays(self,directions,mount=(0,0,.15),minimum=.3,maximum=100.):
        state=self.state();R=rotation(state['orientation']);origin=state['position']+R@np.array(mount)
        world=np.asarray(directions)@R.T
        starts=origin+minimum*world;ends=origin+maximum*world
        result=[]
        for start in range(0,len(world),8192):
            result.extend(self.p.rayTestBatch(starts[start:start+8192].tolist(),ends[start:start+8192].tolist(),
                                             numThreads=1,physicsClientId=self.client))
        distance=np.array([minimum+(maximum-minimum)*hit[2] if hit[0]>=0 and hit[0]!=self.vehicle else np.inf for hit in result])
        return distance

    def close(self):
        if getattr(self,'client',-1)>=0:
            self.p.disconnect(physicsClientId=self.client);self.client=-1

    def __enter__(self):return self
    def __exit__(self,*_):self.close()


class ReferenceController:
    """Ideal-state diagnostic controller, explicitly NOT PX4 or autonomous SLAM.

    A supplied position bias can reproduce GPS-driven compensation in tests.
    Do not use this controller as evidence of estimator/navigation performance.
    """
    def __init__(self,backend):self.backend=backend
    def motors(self,target,position_bias=(0,0,0),observed_position=None,observed_velocity=None):
        b=self.backend;s=b.state();R=rotation(s['orientation'])
        position=s['position'] if observed_position is None else np.asarray(observed_position)
        velocity=s['velocity'] if observed_velocity is None else np.asarray(observed_velocity)
        acceleration=1.5*(np.asarray(target)-position-np.asarray(position_bias))-2.4*velocity
        horizontal=np.linalg.norm(acceleration[:2])
        if horizontal>3:acceleration[:2]*=3/horizontal
        acceleration[2]=np.clip(acceleration[2],-3,3)
        force=b.mass*(acceleration-b.gravity);z=force/np.linalg.norm(force)
        # Fix yaw to ENU east, construct the desired body frame geometrically.
        y=np.cross(z,[1.,0.,0.]);y/=np.linalg.norm(y);desired=np.column_stack([np.cross(y,z),y,z])
        skew=R.T@desired-desired.T@R
        error=.5*np.array([skew[2,1],skew[0,2],skew[1,0]])
        torque=b.inertia*(70*error-16*s['angular_velocity'])
        thrust=max(0.,float(force@R[:,2]));wrench=np.r_[thrust,torque]
        motor_force=np.linalg.solve(b.mixer,wrench)
        return np.sqrt(np.clip(motor_force,0,b.motor_k*np.array([r['maxRotVelocity'] for r in b.rotors])**2)/b.motor_k)
