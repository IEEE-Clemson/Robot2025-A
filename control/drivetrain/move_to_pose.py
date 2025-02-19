""" Move the drivetrain in a trapezoidal motion profile in a linear line.
"""

import math
from time import time
import commands2
from pathplannerlib.commands import PIDConstants
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import (
    TrapezoidProfileRadians,
)

from pathplannerlib.path import (
    PathPlannerPath,
    PathConstraints,
    GoalEndState,
    IdealStartingState,
    RotationTarget,
)

from pathplannerlib.controller import PPHolonomicDriveController

from subsystems.drivetrain import Drivetrain



class TrapezoidalMove(commands2.Command):
    """Move the drivetrain to a given pose using a trapezoidal motion profile.
    """

    def __init__(self, drivetrain: Drivetrain, x: float, y: float, theta: float):
        self._x = x
        self._y = y
        self._theta = theta
        self._drivetrain = drivetrain
        self._trajectory = None
        self._start_time = time()

        self._trajectory_controller = PPHolonomicDriveController(
            PIDConstants(0.5, 1.0, 0, 10),
            PIDConstants(4, 0.7, 0, 10),
        )

        self.addRequirements(drivetrain)
        super().__init__()

    def initialize(self):

        start = self._drivetrain.pose()

        rot = Rotation2d(math.atan2(self._y - start.y, self._x - start.x))
        start = Pose2d(start.x, start.y, rot)
        end = Pose2d(self._x, self._y, rot)
        waypoints = PathPlannerPath.waypointsFromPoses([start, end])
        constraints = PathConstraints(
            0.1, 0.1, 2 * math.pi, 2 * math.pi, unlimited=False
        )
        path = PathPlannerPath(
            waypoints,
            constraints,
            IdealStartingState(
                0.0, self._drivetrain.pose().rotation()
            ),
            GoalEndState(
                0.0, Rotation2d(self._theta)
            ),
        )

        path.preventFlipping = True
        self._trajectory = path.generateTrajectory(
            ChassisSpeeds(0, 0, 0), self._drivetrain.pose().rotation(), self._drivetrain.robot_config
        )

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
        pose_diff = Pose2d(self._x, self._y, self._theta) - self._drivetrain.pose()

        xy_tol = 0.01
        theta_tol = 0.1
        vx, vy, omega = self._drivetrain.get_local_vel()
        return (
            cur_time > self._trajectory.getTotalTimeSeconds()
            and pose_diff.x < xy_tol
            and pose_diff.y < xy_tol
            and pose_diff.rotation().radians() < theta_tol
            and math.hypot(vx, vy) < 0.2
        )

    def end(self, interrupted):
        self._drivetrain.drive_raw_local(0, 0, 0)

    def runsWhenDisabled(self):
        return True
