from math import pi
import math
from time import sleep, time

from smbus2 import SMBus
from control.beacon import extend_beacon, retract_beacon, travel_beacon
from control.dumper import extend_dumper
from control.intake import start_intake, stop_intake
from control.mover import grab_box, release_box
from cu_hal.auxilliary.auxilliary_hw import AuxilliaryHW
from cu_hal.drivetrain.drivetrain_i2c import DrivetrainI2C
from control.drivetrain.move_to_pose import MoveCommand, RamseteMove, move_to_inches
from cu_hal.drivetrain.drivetrain_wifi import DrivetrainWifi
from cu_hal.drivetrain.drivetrain_sim import DrivetrainSim
from cu_hal.interfaces import AuxilliaryHAL
from subsystems.drivetrain import Drivetrain, DrivetrainConfig
from subsystems.dumper import Dumper
from subsystems.intake import Intake
from subsystems.mover import Mover
from subsystems.vision import Vision, VisionConfig
from subsystems.beacon import Beacon
import commands2

# python -m tools.auto_demo

def sweep_gems_trajectory2(drivetrain: Drivetrain):
    x_i = 6
    x_fs = [47, 84, 47]
    y_i = 35.5-6
    dy = -5.5
    cmds = []
    for i in range(len(x_fs)):
        cmd = move_to_inches(drivetrain, x_fs[i], y_i + i*dy, 0, accuracy=0.05, speed=0.4) 
        if i != len(x_fs) - 1:
            cmd = cmd.andThen(move_to_inches(drivetrain, x_i, y_i + i*dy, 0, accuracy=0.05, speed=0.4)) \
            .andThen(move_to_inches(drivetrain, x_i, y_i + (i+1)*dy, 0, accuracy=0.05, speed=0.4))
        cmds.append(cmd)
    return commands2.SequentialCommandGroup(*cmds).ignoringDisable(True)

bus = SMBus(1)
drivetrain_hal = DrivetrainI2C(bus)
drivetrain = Drivetrain(DrivetrainConfig(), drivetrain_hal)
drivetrain.drive_raw_local(0, 0, 0)
drivetrain.periodic()
drivetrain.reset_odom_inches(32.25, 8, -90)

aux_hal = AuxilliaryHW(bus)
beacon = Beacon(aux_hal)
intake = Intake(aux_hal)
dumper = Dumper(aux_hal)
mover = Mover(aux_hal)

sleep(0.5)

drivetrain.reset_odom_inches(0, 0, 0)
auto_command2 = (
    commands2.WaitCommand(1).ignoringDisable(True)
    .andThen(move_to_inches(drivetrain,           24, 0.0,   0, accuracy =0.005))
)
def update_thread():
    t = time()
    dt = 0.02
    commands2.CommandScheduler.getInstance().enable()

    while True:
        t = time()
        commands2.CommandScheduler.getInstance().run()
        t_new = time()
        #print(t_new - t)
        to_sleep = dt - (t_new - t)
        if to_sleep > 0:
            sleep(to_sleep)

auto_command2.schedule()
update_thread()
