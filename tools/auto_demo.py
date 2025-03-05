from math import pi
import math
from time import sleep, time
from cu_hal.drivetrain.drivetrain_i2c import DrivetrainI2C
from control.drivetrain.move_to_pose import MoveCommand, RamseteMove, move_to_inches
from cu_hal.drivetrain.drivetrain_wifi import DrivetrainWifi
from cu_hal.drivetrain.drivetrain_sim import DrivetrainSim
from subsystems.drivetrain import Drivetrain, DrivetrainConfig
from subsystems.vision import Vision, VisionConfig
import commands2

from wpimath.geometry import Pose2d, Rotation2d

from wpimath.trajectory import (
    TrajectoryConfig,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)

# python -m tools.auto_demo

def pose_from_inches(x: float, y: float, theta: float):
    return Pose2d.fromFeet(x / 12, y / 12, Rotation2d.fromDegrees(theta))


def sweep_gems_trajectory(drivetrain: Drivetrain):
    # Should approximately be at 18, 35.5, -90
    start = drivetrain.pose()
    poses = []
    
    # X dist between sweeping patterns
    dx = 9
    # Number of cycles (top to bottom to top)
    n = 2
    # Starting X
    x = 18
    # Bottom Y
    y1 = 5
    # Top Y
    y2 = 40
    # Radius of curves
    r = 9
    for i in range(n):
        poses.append(pose_from_inches(x + 2*i*dx,            y1 + r, -90))
        #poses.append(pose_from_inches(x + 2*i*dx + dx/2,     y1,       0))
        poses.append(pose_from_inches(x + (2*i + 1)*dx,       y1 + r,  90))
        poses.append(pose_from_inches(x + (2*i+1)*dx,        y2 - r,  90))
        #poses.append(pose_from_inches(x + (2*i+1)*dx + dx/2, y2,       0))
        poses.append(pose_from_inches(x + (2*i+2)*dx,        y2 - r, -90))

    end = pose_from_inches(x + (n+1)*dx + dx / 2, y2 - r, 90)
    poses = [pose.translation() for pose in poses]
    config = TrajectoryConfig(0.1, 0.2)
    config.setReversed(False)

    trajectory = TrajectoryGenerator.generateTrajectory(
        start, poses[:-1], end, config
    )
    return trajectory

def sweep_gems_trajectory2(drivetrain: Drivetrain):
    x_i = 18
    x_f = 48
    y_i = 35.5
    dy = -8
    n = 4
    cmds = []
    for i in range(n):
        cmd = move_to_inches(drivetrain, x_f, y_i + i*dy, 0) \
            .andThen(move_to_inches(drivetrain, x_i, y_i + i*dy, 0)) \
            .andThen(move_to_inches(drivetrain, x_i, y_i + (i+1)*dy, 0))
        cmds.append(cmd)
    return commands2.SequentialCommandGroup(*cmds).ignoringDisable(True)

drivetrain_hal = DrivetrainI2C()
drivetrain = Drivetrain(DrivetrainConfig(), drivetrain_hal)
drivetrain.drive_raw_local(0, 0, 0)
drivetrain.periodic()
drivetrain.reset_odom_inches(32.25, 8, -90)

vision_config = VisionConfig()
vision_config.should_display = False
vision_config.dev_index = 0
vision = Vision(vision_config)
vision.add_pose2d_callback = drivetrain.pose_estimator.add_vision_pose
sleep(1)
auto_command = (
        commands2.WaitCommand(1).ignoringDisable(True)
        .andThen(move_to_inches(drivetrain,            32.25, 22.5,   -90))
        .andThen(move_to_inches(drivetrain,  24,    22.5,     0))
        .andThen(move_to_inches(drivetrain,  4,     22.5,     0))
        .andThen(commands2.WaitCommand(3).ignoringDisable(True)) 
        #.andThen(move_to_inches(drivetrain,  47,    22.5,     0))
        #.andThen(move_to_inches(drivetrain,  67,    22.5,     0))
        # Cave
        .andThen(move_to_inches(drivetrain,  84,    22.5,     0))
        .andThen(move_to_inches(drivetrain,  84,    33.5,     0))
        .andThen(move_to_inches(drivetrain,  74,    35.5,     0))
        .andThen(move_to_inches(drivetrain,  74,    11.5,     0))
        .andThen(move_to_inches(drivetrain,  84,    11.5,     0))
        .andThen(move_to_inches(drivetrain,  84,    22.5,     0))

        .andThen(move_to_inches(drivetrain,  43,    22.5,     0))
        .andThen(move_to_inches(drivetrain,  43,    35.5,     0))
        .andThen(move_to_inches(drivetrain,  12,    35.5,     0))
        .andThen(move_to_inches(drivetrain,  12,    32,       0))
        .andThen(sweep_gems_trajectory2(drivetrain))
)

auto_command2 = move_to_inches(drivetrain,            12, 35.5,   0) \
                .andThen(sweep_gems_trajectory2(drivetrain))


def update_thread():
    t = time()
    dt = 0.02
    commands2.CommandScheduler.getInstance().enable()

    while True:
        t = time()
        commands2.CommandScheduler.getInstance().run()
        t_new = time()
        print(t_new - t)
        to_sleep = dt - (t_new - t)
        if to_sleep > 0:
            sleep(to_sleep)


auto_command.schedule()
update_thread()
