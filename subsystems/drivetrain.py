import math
import time
from typing import Tuple
import numpy as np

from commands2 import Subsystem
from pathplannerlib.config import RobotConfig, ModuleConfig
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.system.plant import DCMotor
from wpimath.geometry import Translation2d
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import (
    TrapezoidProfileRadians,
)

from cu_hal.interfaces import DrivetrainHAL
from subsystems.math.poseestimator import PoseEstimator


class DrivetrainConfig:
    pass


class Drivetrain(Subsystem):
    def __init__(self, config: DrivetrainConfig, hal: DrivetrainHAL):
        super().__init__()
        self._config = config
        self._hal = hal

        self._x_odom = np.array([0.794, 5.5*0.0254])
        self._theta_odom = -math.pi/2

        self._target_vx = np.array([0, 0])
        self._target_omega = 0

        self._vx_local = np.array([0, 0])
        self._omega = 0

        self._cur_ref_x_vel = 0
        self._cur_ref_y_vel = 0
        self._cur_ref_omega = 0

        self.slew_rate_xy = 0.75
        self.slew_rate_theta = 3.0
        self.max_speed = 0.1
        self.max_omega = 0.1
        self.pose_estimator = PoseEstimator([0.02, 0.02, 0.02], [0.1, 0.1, 0.1])

        # Config for trajectory controllers
        rx = 0.052
        ry = 0.1175
        # Give the modules absurd amounts of power,
        # the velocity and acceleration should be limited by the trajectory
        module_config = ModuleConfig(0.03, 20.0, 1.2, DCMotor.krakenX60FOC(1), 40, 1)
        self.robot_config = RobotConfig(
            1.0,
            0.1,
            module_config,
            moduleOffsets=[
                Translation2d(rx, ry),
                Translation2d(rx, -ry),
                Translation2d(-rx, ry),
                Translation2d(-rx, -ry),
            ],
        )   
        self.robot_config.moduleConfig.torqueLoss = 0
        self.robot_config.isHolonomic = True

        self.trajectory_controller = HolonomicDriveController(
            PIDController(3, 0.5, 0),
            PIDController(3, 0.5, 0),
            ProfiledPIDControllerRadians(
                4, 0.7, 0, TrapezoidProfileRadians.Constraints(3.14, 6.0)
            ),
        )
        self.prev_time = time.time()

    def reset_odom(self, x: float, y: float, theta: float):
        self._x_odom = np.array([x, y])
        self._theta_odom = theta
        self.pose_estimator.reset_pose(self._x_odom, self._theta_odom)

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
        cur_time = time.time()
        dt = cur_time - self.prev_time  # WPILIB defaults to 50hz
        self.prev_time = cur_time
        super().periodic()

        if abs(self._target_vx[0] - self._cur_ref_x_vel) < self.slew_rate_xy * dt:
            self._cur_ref_x_vel = self._target_vx[0]
        else:
            self._cur_ref_x_vel += (
                self.slew_rate_xy * dt
                if self._target_vx[0] > self._cur_ref_x_vel
                else -self.slew_rate_xy * dt
            )

        if abs(self._target_vx[1] - self._cur_ref_y_vel) < self.slew_rate_xy * dt:
            self._cur_ref_y_vel = self._target_vx[1]
        else:
            self._cur_ref_y_vel += (
                self.slew_rate_xy * dt
                if self._target_vx[1] > self._cur_ref_y_vel
                else -self.slew_rate_xy * dt
            )

        if abs(self._target_omega - self._cur_ref_omega) < self.slew_rate_theta * dt:
            self._cur_ref_omega = self._target_omega
        else:
            self._cur_ref_omega += (
                self.slew_rate_theta * dt
                if self._target_omega > self._cur_ref_omega
                else -self.slew_rate_theta * dt
            )

        t = time.time()
        vx, vy, self._omega = self._hal.set_target_wheel_velocities(
            self._cur_ref_x_vel, self._cur_ref_y_vel, self._cur_ref_omega
        )
        self._vx_local = np.array([vx, vy])

        self.compute_odom(dt)
        self.pose_estimator.update_with_time(
            self._x_odom, self._theta_odom, time.time()
        )

    def drive_raw_local(self, vx, vy, omega):
        if abs(vx) > self.max_speed:
            vx = math.copysign(self.max_speed, vx)
        if abs(vy) > self.max_speed:
            vy = math.copysign(self.max_speed, vy)
        if abs(omega) > self.max_omega:
            omega = math.copysign(self.max_omega, omega)
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

    def pose(self) -> Pose2d:
        return Pose2d(self.pose_x[0], self.pose_x[1], Rotation2d(self.pose_theta))
