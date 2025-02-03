from ast import Tuple
import time
from typing import List
import numpy as np
from hal.interfaces import DrivetrainHAL
from subsystems.math.poseestimator import PoseEstimator

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
        
        self.pose_estimator = PoseEstimator([0.02, 0.02, 0.02], [0.05, 0.05, 0.05])

    def compute_odom(self, dt: float):
        # TODO: Fuse latency compensated vision results with odometry
        x_hat = self._x_odom + self._vx_local * dt * np.array(
            [np.cos(self._theta_odom), np.sin(self._theta_odom)]
        )
        theta_hat = self._theta_odom + self._omega * dt
        self._x_odom = x_hat
        self._theta_odom = theta_hat

    def update(self, dt: float):
        """Updates the subsystem
        Despite dt being a parameter, dt should be as close to a fixed number every time

        Args:
            dt (float): Time since last update
        """
        vx, vy, self._omega = self._hal.set_target_wheel_velocities(
            self._target_vx[0], self._target_vx[1], self._target_omega
        )
        self._vx_local = np.array([vx, vy])

        self.compute_odom(dt)
        self.pose_estimator.update_with_time(time.time_ns())

    def drive_raw_local(self, vx, vy, omega):
        self._target_vx = np.array([vx, vy])
        self._target_omega = omega

    @property
    def pose_x(self):
        return self.pose_estimator.x_est
    

    @property
    def pose_theta(self):
        return self.pose_estimator.theta_est