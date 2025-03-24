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
    PathPlannerTrajectoryState
)


from wpimath.trajectory import (
    TrajectoryConfig,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
    Trajectory
)


from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)

from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.trajectory import PathPlannerTrajectory

from subsystems.drivetrain import Drivetrain


class RamseteMove(commands2.Command):
    """Drives robot along a trajectory given by a lambda, making sure the robot always faces forward"""
    def __init__(self, drivetrain: Drivetrain, trajectory_generator: Callable[[Drivetrain], Trajectory]):
        self._drivetrain = drivetrain
        self._trajectory_generator = trajectory_generator
        self._trajectory = None
        self._start_time = time()
        x_pid = PIDController(4, 0.3, 0)
        y_pid = PIDController(4, 0.3, 0)
        x_pid.setIZone(1)
        y_pid.setIZone(1)

        self._trajectory_controller = HolonomicDriveController(
            x_pid,
            y_pid,
            ProfiledPIDControllerRadians(
                3, 1.0, 2.0, TrapezoidProfileRadians.Constraints(3.14, 6.0)
            ),
        )

        self.addRequirements(drivetrain)
        super().__init__()

    def initialize(self):
        self._trajectory = self._trajectory_generator(self._drivetrain)
        self._start_time = time()

    def execute(self):
        if self._trajectory is None:
            return False
        cur_time = time() - self._start_time
        desired_state = self._trajectory.sample(cur_time)
        speeds = self._trajectory_controller.calculate(self._drivetrain.pose(), desired_state.pose, desired_state.velocity, desired_state.pose.rotation())
        self._drivetrain.drive_raw_local(speeds.vx, speeds.vy, speeds.omega)


    def isFinished(self):
        if self._trajectory is None:
            return False
        cur_time = time() - self._start_time
        pose_diff = self._trajectory.states()[-1].pose - self._drivetrain.pose()

        xy_tol = 0.01
        theta_tol = 0.05
        vx, vy, omega = self._drivetrain.get_local_vel()
        return (
            cur_time > self._trajectory.totalTime()
            and abs(pose_diff.x) < xy_tol
            and abs(pose_diff.y) < xy_tol
            and abs(pose_diff.rotation().radians()) < theta_tol
            and math.hypot(vx, vy) < 0.2
        )

    def end(self, interrupted):
        self._drivetrain.drive_raw_local(0, 0, 0)


    def runsWhenDisabled(self):
        return True

class MoveCommand(commands2.Command):
    """Drives robot along a trajectory given by a lambda"""

    def __init__(
        self,
        drivetrain: Drivetrain,
        trajectory_func: Callable[[Drivetrain], PathPlannerTrajectory],
        accuracy = 0.01
    ):
        self._trajectory_gen = trajectory_func
        self._drivetrain = drivetrain
        self._trajectory = None
        self._start_time = time()
        pid_consts_vxy = PIDConstants(4, 0.4, 0.1, 1)
        self.accuracy = accuracy

        self._trajectory_controller = PPHolonomicDriveController(
            pid_consts_vxy,
            PIDConstants(4, 0.05, 3.0, 1),
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

        theta_tol = 0.05
        vx, vy, omega = self._drivetrain.get_local_vel()
        return (
            cur_time > self._trajectory.getTotalTimeSeconds()
            and abs(pose_diff.x) < self.accuracy
            and abs(pose_diff.y) < self.accuracy
            and abs(pose_diff.rotation().radians()) < theta_tol
            and math.hypot(vx, vy) < 0.2
            and abs(omega) < 0.05
        )

    def end(self, interrupted):
        self._drivetrain.drive_raw_local(0, 0, 0)

    def runsWhenDisabled(self):
        return True


def __trapezoidal_move_trajectory(
    drivetrain: Drivetrain, x: float, y: float, theta: float, speed: float
) -> PathPlannerPath:
    start = drivetrain.pose()

    rot = Rotation2d(math.atan2(y - start.y, x - start.x))
    start = Pose2d(start.x, start.y, rot)
    end = Pose2d(x, y, rot)
    waypoints = PathPlannerPath.waypointsFromPoses([start, end])
    constraints = PathConstraints(speed, 0.5, 1.0, 1*math.pi, unlimited=False)
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
    drivetrain: Drivetrain, x: float, y: float, theta: float, speed = 0.2
) -> MoveCommand:
    return MoveCommand(
        drivetrain,
        lambda drivetrain: __trapezoidal_move_trajectory(drivetrain, x, y, theta, speed),
    )



def move_to_inches(
    drivetrain: Drivetrain, x: float, y: float, theta: float, speed = 0.2, accuracy = 0.01
) -> MoveCommand:
    METERS_TO_INCHES = 39.3701
    return MoveCommand(
        drivetrain,
        lambda drivetrain: __trapezoidal_move_trajectory(drivetrain, x / METERS_TO_INCHES, y / METERS_TO_INCHES, theta * math.pi / 180, speed),
        accuracy = accuracy
    )

