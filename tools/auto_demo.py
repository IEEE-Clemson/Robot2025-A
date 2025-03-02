from math import pi
from threading import Thread
from time import sleep, time
from cu_hal.drivetrain.drivetrain_i2c import DrivetrainI2C
# from comms.commlib import Comm
from control.drivetrain.move_to_pose import TrapezoidalMove
from cu_hal.drivetrain.drivetrain_wifi import DrivetrainWifi
from cu_hal.drivetrain.drivetrain_sim import DrivetrainSim
from subsystems.drivetrain import Drivetrain, DrivetrainConfig
from subsystems.vision import Vision, VisionConfig
import commands2

# def comm_error_handler():
#     print("Error within commlib")



drivetrain_hal = DrivetrainI2C()
drivetrain = Drivetrain(DrivetrainConfig(), drivetrain_hal)
drivetrain.reset_odom(0.794, 5.5*0.0254, -pi/2)
drivetrain.drive_raw_local(0, 0, 0)
drivetrain.periodic()
vision_config = VisionConfig()
vision_config.should_display = False
vision_config.dev_index = 0
vision = Vision(vision_config)
vision.add_pose2d_callback = drivetrain.pose_estimator.add_vision_pose
sleep(1)
auto_command = TrapezoidalMove(drivetrain, 0.794, 1.143 / 2, -pi/2) \
                .andThen(TrapezoidalMove(drivetrain, 1.2, 1.143 / 2, 0)) \
                .andThen(TrapezoidalMove(drivetrain, 1.7, 1.143 / 2, 0)) \
                .andThen(TrapezoidalMove(drivetrain, 1.1, 1.143 / 2, 0)) \
                .andThen(TrapezoidalMove(drivetrain, 1.1, 0.9,  0)) \
                .andThen(TrapezoidalMove(drivetrain, 0.3, 0.9,  0)) \
                #.andThen(commands2.InstantCommand(lambda: drivetrain.reset_odom(0.7, 1.0, 0), drivetrain).ignoringDisable(True)) \
                #.andThen(TrapezoidalMove(drivetrain,s 0.3, 1.0, 0)) \
                #.andThen(TrapezoidalMove(drivetrain, 0.794, 1.143 / 2, 0)) \ # ORIGINALY LINE 2


move_pose_command = TrapezoidalMove(drivetrain, 0.794, 0.58, 0) \
                .andThen(TrapezoidalMove(drivetrain, 1.9, 0.58, 0)) \
                .andThen(TrapezoidalMove(drivetrain, 0.7, 0.58, 0)) \
                .andThen(TrapezoidalMove(drivetrain, 0.7, 1.2,  0)) \
                .andThen(commands2.InstantCommand(lambda: drivetrain.reset_odom(0.7, 1.0, 0), drivetrain).ignoringDisable(True)) \
                .andThen(TrapezoidalMove(drivetrain, 0.3, 1.0, 0)) \



def update_thread():
    t = time()
    dt = 0.02
    commands2.CommandScheduler.getInstance().enable()

    while True:
        t = time()
        commands2.CommandScheduler.getInstance().run()
        print(drivetrain.pose().translation())
        # print(drivetrain._x_odom)
        #print(drivetrain._vx_local, drivetrain._omega)
        t_new = time()
        to_sleep = dt - (t_new - t)
        if to_sleep > 0:
            sleep(to_sleep)
            

#thread = Thread(target=update_thread, daemon=True)
#thread.start()
auto_command.schedule()
update_thread()