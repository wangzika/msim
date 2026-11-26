"""Version-one portable events. All time values are integer nanoseconds.

stamp_ns is acquisition time; arrival_ns is availability time. Replay orders by
arrival time and never rewrites measurement stamps, even for delayed GNSS.
"""
from dataclasses import dataclass
import math

KINDS={'imu','gnss','pose','cloud'}


def vector(value, length, name):
    if not isinstance(value,(list,tuple)) or len(value)!=length or any(type(v) not in (int,float) or not math.isfinite(v) for v in value):
        raise ValueError(f'{name}: expected {length} finite numbers')


@dataclass(frozen=True)
class Event:
    stamp_ns: int
    arrival_ns: int
    kind: str
    stream: str
    frame: str
    data: dict

    def validate(self):
        for key in ('stamp_ns','arrival_ns'):
            if type(getattr(self,key)) is not int or getattr(self,key)<0:raise ValueError(f'Invalid {key}')
        if self.kind not in KINDS:raise ValueError('Unknown event kind')
        if not all(isinstance(v,str) and v and len(v)<=200 for v in (self.stream,self.frame)):raise ValueError('Invalid stream/frame')
        if not isinstance(self.data,dict):raise ValueError('Event data must be an object')
        if self.kind=='imu':
            vector(self.data.get('acceleration_mps2'),3,'acceleration_mps2')
            vector(self.data.get('angular_velocity_radps'),3,'angular_velocity_radps')
            q=self.data.get('orientation_xyzw')
        elif self.kind in ('gnss','pose'):
            vector(self.data.get('position_m'),3,'position_m')
            if self.data.get('point') not in ('body','antenna'):raise ValueError('Position point must be body or antenna')
            q=self.data.get('orientation_xyzw')
        else:
            if not isinstance(self.data.get('file'),str):raise ValueError('Cloud requires a relative payload file')
            if type(self.data.get('point_count')) is not int or self.data['point_count']<0:raise ValueError('Invalid point count')
            q=None
        if q is not None:
            vector(q,4,'orientation_xyzw')
            if abs(sum(x*x for x in q)-1.)>.02:raise ValueError('Orientation must be a unit quaternion')
        return self

    def row(self):
        return dict(schema_version=1,stamp_ns=self.stamp_ns,arrival_ns=self.arrival_ns,
                    kind=self.kind,stream=self.stream,frame=self.frame,data=self.data)

    @classmethod
    def from_row(cls,row):
        if not isinstance(row,dict) or type(row.get('schema_version')) is not int or row['schema_version']!=1:raise ValueError('Unsupported event schema')
        try:return cls(**{key:row[key] for key in ('stamp_ns','arrival_ns','kind','stream','frame','data')}).validate()
        except KeyError as error:raise ValueError('Missing event field '+str(error)) from error
