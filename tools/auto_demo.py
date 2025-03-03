from math import pi
from time import sleep, time
from cu_hal.drivetrain.drivetrain_i2c import DrivetrainI2C
from control.drivetrain.move_to_pose import move_to_inches
from cu_hal.drivetrain.drivetrain_wifi import DrivetrainWifi
from cu_hal.drivetrain.drivetrain_sim import DrivetrainSim
from subsystems.drivetrain import Drivetrain, DrivetrainConfig
from subsystems.vision import Vision, VisionConfig
import commands2

drivetrain_hal = DrivetrainI2C()
drivetrain = Drivetrain(DrivetrainConfig(), drivetrain_hal)
drivetrain.reset_odom_inches(32.25, 5.5, -90)
drivetrain.drive_raw_local(0, 0, 0)
drivetrain.periodic()

vision_config = VisionConfig()
vision_config.should_display = False
vision_config.dev_index = 0
vision = Vision(vision_config)
vision.add_pose2d_callback = drivetrain.pose_estimator.add_vision_pose
sleep(1)
auto_command = move_to_inches(drivetrain,            32.25, 22.5,   -90) \
                .andThen(move_to_inches(drivetrain,  47,    22.5,     0)) \
                .andThen(move_to_inches(drivetrain,  67,    22.5,     0)) \
                .andThen(move_to_inches(drivetrain,  43,    22.5,     0)) \
                .andThen(move_to_inches(drivetrain,  43,    35.5,     0)) \
                .andThen(move_to_inches(drivetrain,  12,    35.5,     0)) \


def update_thread():
    t = time()
    dt = 0.02
    commands2.CommandScheduler.getInstance().enable()

    while True:
        t = time()
        commands2.CommandScheduler.getInstance().run()
        print(drivetrain.pose().translation())
        t_new = time()
        to_sleep = dt - (t_new - t)
        if to_sleep > 0:
            sleep(to_sleep)


auto_command.schedule()
update_thread()
