from ast import Tuple
from typing import List
import numpy as np

from hal.interfaces import DrivetrainHAL

class DrivetrainConfig:
    pass

class Drivetrain:
    def __init__(self, config: DrivetrainConfig, hal: DrivetrainHAL):
        self._config = config
        self._hal = hal

        self._x_odom = np.array([0, 0])
        self._theta_odom = 0

        self._target_vx = np.array([0, 0])
        self._target_omega = 0

        self._vx_local = np.array([0, 0])
        self._omega = 0

        self._xs_vision: List[float]
        self._ys_vision: List[float]
        self._thetas_vision: List[float]
        self.confidences: List[float]

        self._error_covariance = np.zeros((3, 3))

    def compute_odom(self, dt: float):
        # TODO: Fuse latency compensated vision results with odometry
        x_hat = self._x_odom + self._vx_local * dt * np.array([np.cos(self._theta_odom), np.sin(self._theta_odom)])
        theta_hat = self._theta_odom + self._omega * dt
        self._x_odom = x_hat
        self._theta_odom = theta_hat

    def update(self, dt: float):
        """Updates the subsystem
        Despite dt being a parameter, dt should be as close to a fixed number every time

        Args:
            dt (float): Time since last update
        """
        vx, vy, self._omega = self._hal.set_target_wheel_velocities(self._target_vx[0], self._target_vx[1], self._target_omega)
        self._vx_local = np.array([vx, vy])

    def drive_raw_local(self, vx, vy, omega):
        self._target_vx = np.array([vx, vy])
        self._target_omega = omega
