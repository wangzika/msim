"""Small deterministic sensor fixture, not a physics simulator or SLAM run."""
import math
import random
from .recording import RecordingWriter
from .schema import Event


def generate(destination,duration_s=24.,seed=7,attack=True):
    if not math.isfinite(duration_s) or not 4<=duration_s<=600:raise ValueError('Demo duration must be 4..600 s')
    rng=random.Random(seed);start=int(duration_s*.5*1e9)
    metadata=dict(source='synthetic_sensor_fixture',not_a_physics_simulation=True,
        pose_source='precomputed fixture; FAST-LIO is not executed',seed=seed,
        antenna_offset_body_m=[0.,0.,0.],
        labels=dict(attack_start_ns=start if attack else None,scope='evaluation_only'))
    with RecordingWriter(destination,metadata) as writer:
        for i in range(int(duration_s*100)+1):
            t=i/100;stamp=i*10_000_000
            # Smooth N motion after a stationary initialization, with exact
            # body specific force. Position noises use independent draws.
            moving=max(0.,t-3.);north=.15*moving*moving;ay=.3 if t>3 else 0.
            pos=[0.,north,2.];q=[0.,0.,0.,1.]
            writer.write(Event(stamp,stamp,'imu','imu','body_flu',dict(acceleration_mps2=[0.,ay,9.80665],angular_velocity_radps=[0.,0.,0.],orientation_xyzw=q)))
            if i%5==0:writer.write(Event(stamp,stamp,'pose','truth','enu',dict(position_m=pos,orientation_xyzw=q,point='body')))
            if i%10==0:
                lio=[.015*t+rng.gauss(0,.015),north+rng.gauss(0,.02),2.+rng.gauss(0,.01)]
                writer.write(Event(stamp,stamp,'pose','lio','enu',dict(position_m=lio,orientation_xyzw=q,point='body')))
            if i%20==0:
                offset=8.+.2*(t-start/1e9) if attack and stamp>=start else 0.
                gps=[offset+rng.gauss(0,.1),north+rng.gauss(0,.1),2.+rng.gauss(0,.15)]
                writer.write(Event(stamp,stamp,'gnss','gnss','enu',dict(position_m=gps,point='antenna')))
