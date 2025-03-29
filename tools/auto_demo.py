from math import pi
import math
from time import sleep, time

from smbus2 import SMBus
from control.beacon import extend_beacon, retract_beacon, travel_beacon
from control.dumper import extend_dumper
from control.intake import start_intake, stop_intake
from control.mover import grab_box, mover_off, release_box
from control.safety import wait_for_armed, wait_for_led_on
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
from subsystems.safety import Safety
from subsystems.vision import Vision, VisionConfig
from subsystems.beacon import Beacon
import commands2

# python -m tools.auto_demo

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
safety = Safety(aux_hal)


vision_config = VisionConfig()
vision_config.should_display = False
vision_config.dev_index = 0
#vision_config.should_display = True
vision = Vision(vision_config)
vision.add_pose2d_callback = drivetrain.pose_estimator.add_vision_pose
sleep(0.5)

drivetrain.reset_odom_inches(31.5, 6, 0)

def place_nebulite_box(y: float) -> commands2.Command:
    return (
    move_to_inches(drivetrain,            16, y,   0)
    .andThen(release_box(mover))
    .andThen(move_to_inches(drivetrain,  18,     y,     0, speed=0.2))
    .andThen(move_to_inches(drivetrain,  18,     y-5,     0, speed=0.2))
    .andThen(move_to_inches(drivetrain,  15,     y-5,     0, speed=0.2))
    .andThen(extend_dumper(dumper))
    .andThen(move_to_inches(drivetrain,  10,     y-5,     0, speed=0.2))
    )
    

def place_nebulite0() -> commands2.Command:
    y = 7
    return (
    move_to_inches(drivetrain,            11.5, y,   0)
    .andThen(release_box(mover))
    .andThen(move_to_inches(drivetrain,  18, y,     0, speed=0.2))
    .andThen(move_to_inches(drivetrain,  20, 15,     0, speed=0.2))
    .andThen(move_to_inches(drivetrain,  5.5,  12,     90, speed=0.2))
    .andThen(extend_dumper(dumper))
    .andThen(move_to_inches(drivetrain,  5.5,  9,     90, speed=0.4))
    )

def place_nebulite1() -> commands2.Command:
    return place_nebulite_box(4.25 + 5.7 - 3 + 9)
    
def place_nebulite2() -> commands2.Command:
    return place_nebulite_box(4.25 + 5.7 - 3 + 9 * 2)
    
def place_nebulite3() -> commands2.Command:
    return place_nebulite_box(4.25 + 5.7 - 3 + 9 * 3)
    
def place_nebulite4() -> commands2.Command:
    return commands2.WaitCommand(2)
    


def sweep_gems_trajectory2(drivetrain: Drivetrain):
    x_i = 7
    x_fs = [47, 84, 47, 42]
    y_i = 35.5-6
    dy = -7
    cmds = []
    for i in range(len(x_fs)):
        cmd = move_to_inches(drivetrain, x_fs[i], y_i + i*dy, 0, accuracy=0.05, speed=0.4) 
        if i != len(x_fs) - 1:
            cmd = cmd.andThen(move_to_inches(drivetrain, x_i, y_i + i*dy, 0, accuracy=0.05, speed=0.4)) \
            .andThen(move_to_inches(drivetrain, x_i, y_i + (i+1)*dy, 0, accuracy=0.05, speed=0.4))
        cmds.append(cmd)
    return commands2.SequentialCommandGroup(*cmds).ignoringDisable(True)


auto_command2 = (
    wait_for_armed(safety)
    .andThen(commands2.InstantCommand(lambda: drivetrain.reset_odom_inches(32.25, 8, -90)).ignoringDisable(True))
    #.andThen(wait_for_led_on(safety)) # Comment to disable checking for led
    .andThen(start_intake(intake))
    .andThen(move_to_inches(drivetrain,            31.5, 24.0,   180))
    .andThen(move_to_inches(drivetrain,            9, 22.5,   180))
    .andThen(move_to_inches(drivetrain,            18, 24.0,   0))
    .andThen(travel_beacon(beacon))
    .andThen(move_to_inches(drivetrain,  5.75,     24.0,     0, 0.3))
    .andThen(extend_beacon(beacon))
    .andThen(move_to_inches(drivetrain,  10,     24.0,     0, 0.3))
    .andThen(retract_beacon(beacon)) 
    .andThen(move_to_inches(drivetrain,  7,     24.0,     0, 0.3))
    

    .andThen(move_to_inches(drivetrain,  7,      37.0,     0)) # Nebulite box move
    .andThen(move_to_inches(drivetrain,  16,     37.0,     0))
    .andThen(move_to_inches(drivetrain,  16,     32.0,     0))
    .andThen(move_to_inches(drivetrain,  36.5,     32.0,     0))
    .andThen(move_to_inches(drivetrain,  36.5,     38.0,     0))
    .andThen(move_to_inches(drivetrain,  10,     38.0,     0))

    .andThen(move_to_inches(drivetrain,  47,     37.5,     0, speed=0.4)) # Sweep
    .andThen(move_to_inches(drivetrain,  12,     37.5,     0, speed=0.4))
    .andThen(move_to_inches(drivetrain,  12,     30,     0, speed=0.4))
    .andThen(sweep_gems_trajectory2(drivetrain))


    .andThen(move_to_inches(drivetrain,  16,     41.0,     0)) # Dump
    .andThen(move_to_inches(drivetrain,  6,     43.0,     0))
    .andThen(extend_dumper(dumper))

    # Cave
    .andThen(move_to_inches(drivetrain,  40,     22.5,     0, speed=0.4))
    .andThen(move_to_inches(drivetrain,  65,     22.5,     0, speed=0.4))
    .andThen(move_to_inches(drivetrain,  67,     26,     90, speed=0.4)) # sweep start
    .andThen(move_to_inches(drivetrain,  67,     39,     90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  67,     22.5,    90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  73,     22.5,    90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  73,     39,    90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  73,     22.5,    90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  79,     22.5,    90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  79,     39,    90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  79,     22.5,    90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  84,     22.5,    90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  84,     36,    90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  82,     30,    90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  82,     26,    -90, speed=0.4)) # reverse
    .andThen(move_to_inches(drivetrain,  82,     12,       -90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  82,     27,       -90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  79,     27,    -90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  79,     7,     -90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  79,     27,    -90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  73,     27,    -90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  73,     7,    -90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  73,     27,    -90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  70,     27,     -90, speed=0.4)) 
    .andThen(move_to_inches(drivetrain,  70,     7,     -90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  70,     15,    -90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  70,     22.5,    0, speed=0.4))

    .andThen(move_to_inches(drivetrain,  40,     22.5,     0, speed=0.4))

    #Nebulite box
    .andThen(move_to_inches(drivetrain,  47,     15,     90, speed=0.4))
    .andThen(release_box(mover))
    .andThen(move_to_inches(drivetrain,  47,     9,     90, speed=0.4))
    .andThen(grab_box(mover))
    .andThen(move_to_inches(drivetrain,  40,     20,     90, speed=0.4))
    .andThen(move_to_inches(drivetrain,  30,     20,     0, speed=0.2))
    .andThen(commands2.WaitCommand(0.5))
    .andThen(place_nebulite0().onlyIf(lambda: vision.get_telemetry() == 0))
    .andThen(place_nebulite1().onlyIf(lambda: vision.get_telemetry() == 1))
    .andThen(place_nebulite2().onlyIf(lambda: vision.get_telemetry() == 2 or vision.get_telemetry() is None))
    .andThen(place_nebulite3().onlyIf(lambda: vision.get_telemetry() == 3))
    .andThen(place_nebulite4().onlyIf(lambda: vision.get_telemetry() == 4))

    .andThen(stop_intake(intake))
    .andThen(mover_off(mover))
).ignoringDisable(True)
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
