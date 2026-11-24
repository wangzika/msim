"""Keyboard velocity conversion and input expiry, independent of ROS and Qt."""
import math
from collections import deque
import threading
import time

MOVEMENT_KEYS = frozenset('wasdqerf')


class CommandBuffer:
    """Coalesce pending movement snapshots without dropping discrete actions."""
    def __init__(self):
        self.pending=deque();self.lock=threading.Lock()

    def put(self,command):
        item=(dict(command),time.monotonic())
        with self.lock:
            if command.get('op') in ('keys','hold') and self.pending and self.pending[-1][0].get('op') in ('keys','hold'):
                self.pending[-1]=item
            else:self.pending.append(item)

    def drain(self):
        with self.lock:
            items=list(self.pending);self.pending.clear()
        return items


def body_velocity(keys, yaw, speed=2., vertical=.8, turn=math.pi/6):
    """Body forward/right/up inputs -> world NED velocity and yaw rate."""
    if not all(math.isfinite(x) for x in (yaw, speed, vertical, turn)):
        raise ValueError('Control values must be finite')
    if not 0 < speed <= 10 or not 0 < vertical <= 3 or not 0 < turn <= math.pi:
        raise ValueError('Control speed is outside the supported range')
    forward = int('w' in keys)-int('s' in keys)
    right = int('d' in keys)-int('a' in keys)
    norm = max(1., math.hypot(forward, right))
    forward, right = speed*forward/norm, speed*right/norm
    return (math.cos(yaw)*forward-math.sin(yaw)*right,
            math.sin(yaw)*forward+math.cos(yaw)*right,
            vertical*(int('f' in keys)-int('r' in keys)),
            turn*(int('e' in keys)-int('q' in keys)))


class KeyInput:
    def __init__(self, timeout=.35):
        self.timeout = timeout
        self.keys = frozenset()
        self.updated = -math.inf

    def update(self, keys, now):
        if not isinstance(keys, (list, tuple, set, frozenset)) or any(k not in MOVEMENT_KEYS for k in keys):
            raise ValueError('Expected movement keys W/S/A/D/Q/E/R/F')
        self.keys = frozenset(keys)
        self.updated = now

    def active(self, now):
        return self.keys if 0 <= now-self.updated <= self.timeout else frozenset()

    def clear(self):
        self.keys = frozenset()


def trajectory_window(first_ns, last_ns):
    """Whole-second usable duration after the same warmup as fixed captures."""
    start = ((first_ns+1_000_000_000+199_999_999)//200_000_000)*200_000_000
    duration = (last_ns-start)//1_000_000_000
    if duration < 2:
        raise ValueError('Recording is too short for trajectory export; keep the raw bag')
    return start, int(duration)
