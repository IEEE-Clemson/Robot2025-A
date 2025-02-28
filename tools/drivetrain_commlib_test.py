import time
from cu_hal.drivetrain.drivetrain_hw import DrivetrainHW
from comms.commlib import Comm

def comm_error_handler():
    print("Error within commlib")

comm = Comm('CONTROL', comm_error_handler)

drivetrain_hal = DrivetrainHW(comm)

while True:
    print(drivetrain_hal.get_local_velocity())
    print(drivetrain_hal.set_target_wheel_velocities(0.1, 0.1, 0.1))
    time.sleep(1)