from typing import Tuple
from cu_hal.interfaces import DrivetrainHAL


class DrivetrainSim(DrivetrainHAL):
    """Implementation of the drivetrain HAL for physical hardware using wifi
    """
    def __init__(self):
        super().__init__()

    def get_local_velocity(self) -> Tuple[float, float, float]:
        return 0, 0, 0

    def set_target_wheel_velocities(self, vx: float, vy: float, omega: float) -> Tuple[float, float, float]:
        return vx, vy, omega
    