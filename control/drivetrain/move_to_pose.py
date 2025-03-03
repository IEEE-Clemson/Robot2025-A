""" Move the drivetrain in a trapezoidal motion profile in a linear line.
"""

import math
from time import time
from typing import Callable
import commands2
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

from pathplannerlib.commands import PIDConstants
from pathplannerlib.path import (
    PathPlannerPath,
    PathConstraints,
    GoalEndState,
    IdealStartingState,
)

from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.trajectory import PathPlannerTrajectory

from subsystems.drivetrain import Drivetrain


class MoveCommand(commands2.Command):
    """Drives robot along a trajectory given by a lambda"""

    def __init__(
        self,
        drivetrain: Drivetrain,
        trajectory_func: Callable[[Drivetrain], PathPlannerTrajectory],
    ):
        self._trajectory_gen = trajectory_func
        self._drivetrain = drivetrain
        self._trajectory = None
        self._start_time = time()
        pid_consts_vxy = PIDConstants(4, 0.3, 0, 1)

        self._trajectory_controller = PPHolonomicDriveController(
            pid_consts_vxy,
            PIDConstants(5, 1.0, 0.5, 3),
        )

        self.addRequirements(drivetrain)
        super().__init__()

    def initialize(self):
        self._trajectory = self._trajectory_gen(self._drivetrain)

        self._start_time = time()

    def execute(self):
        if self._trajectory is None:
            return
        cur_time = time() - self._start_time
        desired_state = self._trajectory.sample(cur_time)
        speeds = self._trajectory_controller.calculateRobotRelativeSpeeds(
            self._drivetrain.pose(),
            desired_state,
        )
        self._drivetrain.drive_raw_local(speeds.vx, speeds.vy, speeds.omega)

    def isFinished(self):
        if self._trajectory is None:
            return False
        cur_time = time() - self._start_time
        pose_diff = self._trajectory.getEndState().pose - self._drivetrain.pose()

        xy_tol = 0.01
        theta_tol = 0.05
        vx, vy, omega = self._drivetrain.get_local_vel()
        return (
            cur_time > self._trajectory.getTotalTimeSeconds()
            and abs(pose_diff.x) < xy_tol
            and abs(pose_diff.y) < xy_tol
            and abs(pose_diff.rotation().radians()) < theta_tol
            and math.hypot(vx, vy) < 0.2
        )

    def end(self, interrupted):
        self._drivetrain.drive_raw_local(0, 0, 0)

    def runsWhenDisabled(self):
        return True


def __trapezoidal_move_trajectory(
    drivetrain: Drivetrain, x: float, y: float, theta: float
) -> PathPlannerPath:
    start = drivetrain.pose()

    rot = Rotation2d(math.atan2(y - start.y, x - start.x))
    start = Pose2d(start.x, start.y, rot)
    end = Pose2d(x, y, rot)
    waypoints = PathPlannerPath.waypointsFromPoses([start, end])
    constraints = PathConstraints(0.3, 0.3, 2 * math.pi, 2 * math.pi, unlimited=False)
    path = PathPlannerPath(
        waypoints,
        constraints,
        IdealStartingState(0.0, drivetrain.pose().rotation()),
        GoalEndState(0.0, Rotation2d(theta)),
    )

    path.preventFlipping = True
    trajectory = path.generateTrajectory(
        ChassisSpeeds(0, 0, 0), drivetrain.pose().rotation(), drivetrain.robot_config
    )

    return trajectory


def move_to_meters(
    drivetrain: Drivetrain, x: float, y: float, theta: float
) -> MoveCommand:
    return MoveCommand(
        drivetrain,
        lambda drivetrain: __trapezoidal_move_trajectory(drivetrain, x, y, theta),
    )



def move_to_inches(
    drivetrain: Drivetrain, x: float, y: float, theta: float
) -> MoveCommand:
    METERS_TO_INCHES = 39.3701
    return MoveCommand(
        drivetrain,
        lambda drivetrain: __trapezoidal_move_trajectory(drivetrain, x / METERS_TO_INCHES, y / METERS_TO_INCHES, theta / math.pi * 180),
    )

