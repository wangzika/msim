"""Deterministic, unit-aware MEMS IMU error model (no ROS dependency).

Noise densities are continuous-time values. A sample averaged over dt has
standard deviation density / sqrt(dt). Bias random walks are driven by an
Ornstein-Uhlenbeck process with the configured correlation time.
"""

from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class ImuConfig:
    rate_hz: float
    seed: int
    gyro_noise_density: float
    accel_noise_density: float
    gyro_turn_on_bias_sigma: float
    accel_turn_on_bias_sigma: float
    gyro_bias_random_walk: float
    accel_bias_random_walk: float
    gyro_bias_correlation_time_s: float
    accel_bias_correlation_time_s: float
    orientation_initial_sigma_rad: float
    orientation_random_walk_rad_sqrt_s: float
    orientation_white_sigma_rad: float

    @classmethod
    def from_dict(cls, values):
        config = cls(**{name: values[name] for name in cls.__dataclass_fields__})
        for name, value in vars(config).items():
            if name == 'seed':
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError('seed must be an integer')
            elif not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
            elif (name == 'rate_hz' or name.endswith('_correlation_time_s')) and value <= 0:
                raise ValueError(f'{name} must be positive')
            elif value < 0:
                raise ValueError(f'{name} must be nonnegative')
        return config


def _unit_quaternion(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('orientation must be a finite xyzw quaternion')
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-8:
        raise ValueError('orientation quaternion is zero')
    return tuple(v / norm for v in q)


def _multiply_quaternion(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _small_angle_quaternion(error):
    angle = math.sqrt(sum(v * v for v in error))
    if angle < 1e-12:
        return (error[0] / 2, error[1] / 2, error[2] / 2, 1.0)
    scale = math.sin(angle / 2) / angle
    return (error[0] * scale, error[1] * scale,
            error[2] * scale, math.cos(angle / 2))


class ImuNoiseModel:
    def __init__(self, config):
        self.config = config
        self.rng = random.Random(config.seed)
        self.last_stamp_s = None
        self.elapsed_s = 0.0
        self.gyro_bias = [self.rng.gauss(0, config.gyro_turn_on_bias_sigma)
                          for _ in range(3)]
        self.accel_bias = [self.rng.gauss(0, config.accel_turn_on_bias_sigma)
                           for _ in range(3)]
        self.orientation_error = [self.rng.gauss(0, config.orientation_initial_sigma_rad)
                                  for _ in range(3)]

    def _advance_bias(self, bias, random_walk, correlation_time, dt):
        phi = math.exp(-dt / correlation_time)
        sigma = random_walk * math.sqrt(correlation_time / 2 * (1 - phi * phi))
        for axis in range(3):
            bias[axis] = phi * bias[axis] + self.rng.gauss(0, sigma)

    def sample(self, stamp_s, angular_velocity, linear_acceleration, orientation):
        if not math.isfinite(stamp_s):
            raise ValueError('timestamp must be finite')
        if self.last_stamp_s is not None and stamp_s <= self.last_stamp_s:
            raise ValueError('IMU timestamps must increase')
        for name, vector in (('angular_velocity', angular_velocity),
                             ('linear_acceleration', linear_acceleration)):
            if len(vector) != 3 or not all(math.isfinite(v) for v in vector):
                raise ValueError(f'{name} must be a finite 3-vector')
        orientation = _unit_quaternion(orientation)
        dt = (1 / self.config.rate_hz if self.last_stamp_s is None
              else stamp_s - self.last_stamp_s)
        self.last_stamp_s = stamp_s
        self.elapsed_s += dt
        cfg = self.config
        self._advance_bias(self.gyro_bias, cfg.gyro_bias_random_walk,
                           cfg.gyro_bias_correlation_time_s, dt)
        self._advance_bias(self.accel_bias, cfg.accel_bias_random_walk,
                           cfg.accel_bias_correlation_time_s, dt)
        gyro_sigma = cfg.gyro_noise_density / math.sqrt(dt)
        accel_sigma = cfg.accel_noise_density / math.sqrt(dt)
        gyro = tuple(angular_velocity[i] + self.gyro_bias[i] +
                     self.rng.gauss(0, gyro_sigma) for i in range(3))
        accel = tuple(linear_acceleration[i] + self.accel_bias[i] +
                      self.rng.gauss(0, accel_sigma) for i in range(3))
        attitude_walk_sigma = cfg.orientation_random_walk_rad_sqrt_s * math.sqrt(dt)
        for axis in range(3):
            self.orientation_error[axis] += self.rng.gauss(0, attitude_walk_sigma)
        attitude_error = [value + self.rng.gauss(0, cfg.orientation_white_sigma_rad)
                          for value in self.orientation_error]
        attitude = _unit_quaternion(_multiply_quaternion(
            orientation, _small_angle_quaternion(attitude_error)))
        attitude_variance = (cfg.orientation_initial_sigma_rad ** 2 +
                             cfg.orientation_random_walk_rad_sqrt_s ** 2 * self.elapsed_s +
                             cfg.orientation_white_sigma_rad ** 2)
        return dict(angular_velocity=gyro, linear_acceleration=accel,
                    orientation=attitude, gyro_variance=gyro_sigma ** 2,
                    accel_variance=accel_sigma ** 2,
                    orientation_variance=attitude_variance,
                    gyro_bias=tuple(self.gyro_bias), accel_bias=tuple(self.accel_bias))
