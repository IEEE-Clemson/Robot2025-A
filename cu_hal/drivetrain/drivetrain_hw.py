from typing import Tuple
import comms
from cu_hal.interfaces import DrivetrainHAL


class DrivetrainHW(DrivetrainHAL):
    """Implementation of the drivetrain HAL for physical hardware using UART communication
    """
    def __init__(self, comm: comms.Comm):
        super().__init__()
        self.__comm = comm

    def get_local_velocity(self) -> Tuple[float, float, float]:
        res, local_vx, local_vy, local_omega = self.__comm.send('NAVIGATION', 'STATUS')
        return local_vx, local_vy, local_omega

    def set_target_wheel_velocities(self, vx: float, vy: float, omega: float) -> Tuple[float, float, float]:
        res, local_vx, local_vy, local_omega = self.__comm.send('NAVIGATION', 'MOVE FORWARD', vx, vy, omega)
        return local_vx, local_vy, local_omega
    
    
    