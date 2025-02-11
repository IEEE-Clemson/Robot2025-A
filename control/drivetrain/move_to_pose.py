""" Move the drivetrain in a trapezoidal motion profile in a linear line.
"""

import math
from time import time
import commands2
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds


from pathplannerlib.path import (
    PathPlannerPath,
    PathConstraints,
    GoalEndState,
    IdealStartingState,
    RotationTarget,
)


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


        self.addRequirements(drivetrain)
        super().__init__()

    def initialize(self):

        start = self._drivetrain.pose()

        rot = Rotation2d(math.atan2(self._y - start.y, self._x - start.x))
        start = Pose2d(start.x, start.y, rot)
        end = Pose2d(self._x, self._y, rot)
        waypoints = PathPlannerPath.waypointsFromPoses([start, end])
        constraints = PathConstraints(
            0.1, 0.5, 2 * math.pi, 2 * math.pi, unlimited=False
        )
        path = PathPlannerPath(
            waypoints,
            constraints,
            IdealStartingState(
                0.0, start.rotation()
            ),
            GoalEndState(
                0.0, Rotation2d(self._theta)
            ),
            holonomic_rotations=[
                RotationTarget(0.1, start.rotation()),
                RotationTarget(1.0, Rotation2d(self._theta)),
            ],
        )

        path.preventFlipping = True
        self._trajectory = path.generateTrajectory(
            ChassisSpeeds(0, 0, 0), start.rotation(), self._drivetrain.robot_config
        )
        for state in self._trajectory.getStates():

            print(state.timeSeconds)
            print(state.pose.translation())

        self._start_time = time()

    def execute(self):
        if self._trajectory is None:
            return
        cur_time = time() - self._start_time
        desired_state = self._trajectory.sample(cur_time)
        speeds = self._drivetrain.trajectory_controller.calculate(
            self._drivetrain.pose(),
            desired_state.pose,
            desired_state.linearVelocity,
            desired_state.pose.rotation(),
        )
        self._drivetrain.drive_raw_local(speeds.vx, speeds.vy, speeds.omega)

    def isFinished(self):
        if self._trajectory is None:
            return False
        cur_time = time() - self._start_time
        pose_diff = Pose2d(self._x, self._y, self._theta) - self._drivetrain.pose()

        xy_tol = 0.01
        theta_tol = 0.1
        return (
            cur_time > self._trajectory.getTotalTimeSeconds()
            and pose_diff.x < xy_tol
            and pose_diff.y < xy_tol
            and pose_diff.rotation().radians() < theta_tol
        )

    def end(self, interrupted):
        self._drivetrain.drive_raw_local(0, 0, 0)

    def runsWhenDisabled(self):
        return True
