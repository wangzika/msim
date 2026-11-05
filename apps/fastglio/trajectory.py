"""Versioned Gazebo truth trajectory validation and interpolation."""
import json
import math
from pathlib import Path


def finite_vector(value, length, name):
    if not isinstance(value, list) or len(value) != length or any(
        type(x) not in (int, float) or not math.isfinite(x) for x in value
    ):
        raise ValueError(f'{name} must be {length} finite numbers')


def validate(document, max_gap_ns=20_000_000):
    if document.get('schema_version') != 1 or document.get('frame') != 'map':
        raise ValueError('trajectory requires schema_version 1 and map frame')
    lever = document.get('antenna_offset_base_m')
    finite_vector(lever, 3, 'antenna_offset_base_m')
    samples = document.get('samples')
    if not isinstance(samples, list) or len(samples) < 2:
        raise ValueError('trajectory requires at least two samples')
    previous = None
    for index, sample in enumerate(samples):
        stamp = sample.get('sim_time_ns')
        if type(stamp) is not int or stamp < 0:
            raise ValueError(f'samples[{index}].sim_time_ns invalid')
        if previous is not None and (stamp <= previous or stamp - previous > max_gap_ns):
            raise ValueError(f'samples[{index}] timestamp duplicate, reversed or gap too large')
        for name, size in (('position_ecef_m', 3), ('velocity_ecef_mps', 3),
                           ('orientation_earth_base_xyzw', 4), ('angular_velocity_body_radps', 3)):
            finite_vector(sample.get(name), size, f'samples[{index}].{name}')
        q = sample['orientation_earth_base_xyzw']
        if abs(math.sqrt(sum(x*x for x in q)) - 1) > 1e-3:
            raise ValueError(f'samples[{index}] non-unit orientation')
        previous = stamp
    return document


def load(path):
    return validate(json.loads(Path(path).read_text()))


def antenna_state(document, stamp, stamps=None):
    """Interpolate sampled position/velocity, applying a rotating rigid lever arm."""
    from bisect import bisect_left
    samples = document['samples']
    if stamps is None:
        stamps = [sample['sim_time_ns'] for sample in samples]
    index = bisect_left(stamps, stamp)
    if index == len(samples) or stamp < stamps[0]:
        raise ValueError('epoch outside recorded trajectory')
    if stamps[index] == stamp:
        left = right = samples[index]
        weight = 0.0
    else:
        left, right = samples[index - 1], samples[index]
        weight = (stamp - left['sim_time_ns']) / (right['sim_time_ns'] - left['sim_time_ns'])
    def blend(name):
        return [a + weight * (b - a) for a, b in zip(left[name], right[name])]
    position = blend('position_ecef_m')
    velocity = blend('velocity_ecef_mps')
    first_q = left['orientation_earth_base_xyzw']
    second_q = right['orientation_earth_base_xyzw']
    if sum(a*b for a, b in zip(first_q, second_q)) < 0:
        second_q = [-x for x in second_q]
    quaternion = [a + weight * (b - a) for a, b in zip(first_q, second_q)]
    norm = math.sqrt(sum(x*x for x in quaternion))
    if norm == 0:
        raise ValueError('orientation interpolation is singular')
    quaternion = [x / norm for x in quaternion]
    x, y, z, w = quaternion
    offset = document['antenna_offset_base_m']
    cross = [y*offset[2]-z*offset[1], z*offset[0]-x*offset[2], x*offset[1]-y*offset[0]]
    rotated = [offset[i] + 2*(w*cross[i] + [y*cross[2]-z*cross[1], z*cross[0]-x*cross[2], x*cross[1]-y*cross[0]][i]) for i in range(3)]
    omega = blend('angular_velocity_body_radps')
    lever_velocity = [omega[1]*offset[2]-omega[2]*offset[1],
                      omega[2]*offset[0]-omega[0]*offset[2],
                      omega[0]*offset[1]-omega[1]*offset[0]]
    cross_v = [y*lever_velocity[2]-z*lever_velocity[1], z*lever_velocity[0]-x*lever_velocity[2], x*lever_velocity[1]-y*lever_velocity[0]]
    rotated_v = [lever_velocity[i] + 2*(w*cross_v[i] + [y*cross_v[2]-z*cross_v[1], z*cross_v[0]-x*cross_v[2], x*cross_v[1]-y*cross_v[0]][i]) for i in range(3)]
    return [position[i] + rotated[i] for i in range(3)], [velocity[i] + rotated_v[i] for i in range(3)]
