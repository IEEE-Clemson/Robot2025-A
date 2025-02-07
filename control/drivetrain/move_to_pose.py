""" Move the drivetrain in a trapezoidal motion profile in a linear line.
"""

from time import time
import commands2
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import (
    TrajectoryConfig,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory.constraint import MecanumDriveKinematicsConstraint

from subsystems.drivetrain import Drivetrain


class TrapezoidalMove(commands2.Command):

    def __init__(self, drivetrain: Drivetrain, x: float, y: float, theta: float):
        self._x = x
        self._y = y
        self._theta = theta
        self._drivetrain = drivetrain
        self._trajectory = None
        self._start_time = time()

        self._controller = HolonomicDriveController(
            PIDController(1, 0, 0),
            PIDController(1, 0, 0),
            ProfiledPIDControllerRadians(
                4, 0.4, 0, TrapezoidProfileRadians.Constraints(3.14, 6.0)
            ),
        )
        self.addRequirements(drivetrain)
        super().__init__()

    def initialize(self):

        sideStart = self._drivetrain.pose()
        crossScale = Pose2d(self._x, self._y, self._theta)

        config = TrajectoryConfig(0.2, 0.5)
        config.setReversed(False)


        self._trajectory = TrajectoryGenerator.generateTrajectory(
            sideStart, [], crossScale, config
        )

        self._start_time = time()

    def execute(self):
        print("Here")
        if self._trajectory is None:
            return False
        cur_time = time() - self._start_time
        desired_state = self._trajectory.sample(cur_time)
        speeds = self._controller.calculate(
            self._drivetrain.pose(),
            desired_state,
            desired_state.pose.rotation(),
        )
        self._drivetrain.drive_raw_local(speeds.vx, speeds.vy, speeds.omega)

    def isFinished(self):
        if self._trajectory is None:
            return False
        cur_time = time() - self._start_time
        pose_diff = Pose2d(self._x, self._y, self._theta) - self._drivetrain.pose()

        return cur_time > self._trajectory.totalTime() and pose_diff.x < 0.1 and  pose_diff.y < 0.1 and  pose_diff.rotation().radians() < 0.1

    def end(self, interrupted):
        self._drivetrain.drive_raw_local(0, 0, 0)

    def runsWhenDisabled(self):
        return True
