"""Planar IMU/GNSS innovation detector inspired by Thesis_2020's chi-square test.

The detector consumes only IMU acceleration and the GPS position sent to PX4.
Ground truth and the configured attack are never used in its state update.
"""

import math
from collections import deque

import numpy as np


CHI2_2_9999 = -2.0 * math.log(0.0001)


def horizontal_acceleration_values(acceleration, quaternion):
    """Body FLU specific force rotated into ENU; gravity has no XY component."""
    if quaternion is None:return None
    x,y,z,w=quaternion;vx,vy,vz=acceleration
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    if not .9<=norm<=1.1:return None
    x,y,z,w=x/norm,y/norm,z/norm,w/norm
    return ((1-2*(y*y+z*z))*vx+2*(x*y-z*w)*vy+2*(x*z+y*w)*vz,
            2*(x*y+z*w)*vx+(1-2*(x*x+z*z))*vy+2*(y*z-x*w)*vz)


class PlanarInnovationDetector:
    def __init__(self, gps_sigma_m=0.7, acceleration_sigma_mps2=0.25):
        self.r = gps_sigma_m ** 2
        self.acc_variance = acceleration_sigma_mps2 ** 2
        self.x = None  # east, north, east velocity, north velocity
        self.p = None
        self.last_time_ns = None
        self.innovations = deque(maxlen=10)
        self.shadow_x = None
        self.shadow_p = None
        self.shadow_started_ns = None
        self.samples = 0

    def step(self, time_ns, east_m, north_m, acceleration_enu=None):
        position = np.array([east_m, north_m], dtype=float)
        if not np.all(np.isfinite(position)):
            raise ValueError('non-finite GPS position')
        if self.x is None:
            self.x = np.array([east_m, north_m, 0., 0.])
            self.p = np.diag([self.r, self.r, 4., 4.])
            self.last_time_ns = time_ns
            return self._result(time_ns, position, None, None, 'initializing')
        dt = (time_ns - self.last_time_ns) / 1e9
        if dt <= 0:
            raise ValueError('GPS simulation time must increase')
        if dt > 1.0:
            self.__init__(math.sqrt(self.r), math.sqrt(self.acc_variance))
            return self.step(time_ns, east_m, north_m, acceleration_enu)
        self.last_time_ns = time_ns
        ax, ay = acceleration_enu if acceleration_enu is not None else (0., 0.)
        if not math.isfinite(ax) or not math.isfinite(ay):
            ax = ay = 0.
        a = np.array([ax, ay])
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        b = np.array([[0.5*dt*dt, 0.], [0., 0.5*dt*dt], [dt, 0.], [0., dt]])
        self.x = f @ self.x + b @ a
        self.p = f @ self.p @ f.T + self.acc_variance * b @ b.T
        if self.shadow_x is not None:
            self.shadow_x = f @ self.shadow_x + b @ a
            self.shadow_p = f @ self.shadow_p @ f.T + self.acc_variance * b @ b.T
        residual = position - self.x[:2]
        s = self.p[:2, :2] + np.eye(2) * self.r
        q = float(residual @ np.linalg.solve(s, residual))
        # A short window exposes slow drift. Its covariance assumes independent
        # innovations, so the window score is diagnostic rather than calibrated.
        self.innovations.append((residual.copy(), s.copy()))
        mean = np.mean([item[0] for item in self.innovations], axis=0)
        mean_s = sum((item[1] for item in self.innovations)) / len(self.innovations)**2
        window_q = float(mean @ np.linalg.solve(mean_s, mean))
        inertial_q = None
        if self.shadow_x is not None:
            shadow_residual = position - self.shadow_x[:2]
            shadow_s = self.shadow_p[:2, :2] + np.eye(2) * self.r
            inertial_q = float(shadow_residual @ np.linalg.solve(shadow_s, shadow_residual))
        alarm = (q > CHI2_2_9999 or
                 (len(self.innovations) == 10 and window_q > CHI2_2_9999) or
                 (inertial_q is not None and inertial_q > CHI2_2_9999))
        if not alarm:
            k = self.p[:, :2] @ np.linalg.inv(s)
            self.x += k @ residual
            i_kh = np.eye(4)
            i_kh[:, :2] -= k
            self.p = i_kh @ self.p @ i_kh.T + self.r * k @ k.T
            self.samples += 1
            if self.samples >= 15 and (self.shadow_x is None or
                                       time_ns - self.shadow_started_ns >= 5_000_000_000):
                self.shadow_x = self.x.copy()
                self.shadow_p = self.p.copy()
                self.shadow_p[2, 2] = max(self.shadow_p[2, 2], 0.09)
                self.shadow_p[3, 3] = max(self.shadow_p[3, 3], 0.09)
                self.shadow_started_ns = time_ns
        result = self._result(time_ns, position, q, window_q,
                              'alarm' if alarm else 'normal')
        result['inertial_chi2'] = inertial_q
        return result

    def _result(self, time_ns, position, q, window_q, status):
        return dict(sim_time_ns=time_ns, gps_east_m=float(position[0]),
                    gps_north_m=float(position[1]), estimate_east_m=float(self.x[0]),
                    estimate_north_m=float(self.x[1]), innovation_chi2=q,
                    window_chi2=window_q, inertial_chi2=None,
                    threshold=CHI2_2_9999,
                    detector_status=status)
