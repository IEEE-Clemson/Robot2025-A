import time
from typing import List, Tuple
import numpy as np
from cu_hal.interfaces import DrivetrainHAL
from subsystems.math.poseestimator import PoseEstimator
from commands2 import Subsystem


class DrivetrainConfig:
    pass


class Drivetrain(Subsystem):
    def __init__(self, config: DrivetrainConfig, hal: DrivetrainHAL):
        super().__init__()
        self._config = config
        self._hal = hal

        self._x_odom = np.array([0, 0])
        self._theta_odom = 0

        self._target_vx = np.array([0, 0])
        self._target_omega = 0

        self._vx_local = np.array([0, 0])
        self._omega = 0

        self._cur_ref_x_vel = 0
        self._cur_ref_y_vel = 0
        self._cur_ref_omega = 0

        self.slew_rate_xy = 0.5
        self.slew_rate_theta = 3.0

        self.pose_estimator = PoseEstimator([0.02, 0.02, 0.02], [0.05, 0.05, 0.05])

    def compute_odom(self, dt: float):
        # TODO: Fuse latency compensated vision results with odometry
        x_hat = (
            self._x_odom
            + np.array(
                [
                    [np.cos(self._theta_odom), -np.sin(self._theta_odom)],
                    [np.sin(self._theta_odom), np.cos(self._theta_odom)],
                ]
            )
            @ self._vx_local
            * dt
        )
        theta_hat = self._theta_odom + self._omega * dt
        self._x_odom = x_hat
        self._theta_odom = theta_hat


    def periodic(self):
        """Updates the subsystem
        Despite dt being a parameter, dt should be as close to a fixed number every time

        """
        dt = 0.02 # WPILIB defaults to 50hz
        super().periodic()

        if abs(self._target_vx[0] - self._cur_ref_x_vel) < self.slew_rate_xy * dt:
            self._cur_ref_x_vel = self._target_vx[0]
        else:
            self._cur_ref_x_vel += self.slew_rate_xy * dt if self._target_vx[0] > self._cur_ref_x_vel else -self.slew_rate_xy * dt

        if abs(self._target_vx[1] - self._cur_ref_y_vel) < self.slew_rate_xy * dt:
            self._cur_ref_y_vel = self._target_vx[1]
        else:
            self._cur_ref_y_vel += self.slew_rate_xy * dt if self._target_vx[1] > self._cur_ref_y_vel else -self.slew_rate_xy * dt

        if abs(self._target_omega - self._cur_ref_omega) < self.slew_rate_theta * dt:
            self._cur_ref_omega = self._target_omega
        else:
            self._cur_ref_omega += self.slew_rate_theta * dt if self._target_omega > self._cur_ref_omega else -self.slew_rate_theta * dt


        vx, vy, self._omega = self._hal.set_target_wheel_velocities(
            self._cur_ref_x_vel, self._cur_ref_y_vel, self._cur_ref_omega
        )
        self._vx_local = np.array([vx, vy])

        self.compute_odom(dt)
        self.pose_estimator.update_with_time(
            self._x_odom, self._theta_odom, time.time_ns()
        )

    def drive_raw_local(self, vx, vy, omega):
        self._target_vx = np.array([vx, vy])
        self._target_omega = omega

    def get_local_vel(self) -> Tuple[float, float, float]:
        return self._vx_local[0], self._vx_local[1], self._omega

    @property
    def pose_x(self):
        return self.pose_estimator.x_est

    @property
    def pose_theta(self):
        return self.pose_estimator.theta_est
