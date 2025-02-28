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
        self.__comm.send('NAVIGATION', 'STATUS')
        msg = self.__comm.receive()
        res, local_vx, local_vy, local_omega = msg.arguments
        return local_vx, local_vy, local_omega

    def set_target_wheel_velocities(self, vx: float, vy: float, omega: float) -> Tuple[float, float, float]:
        self.__comm.send('NAVIGATION', 'MOVE FORWARD', float(vx), float(vy), float(omega))
        msg = self.__comm.receive()
        res, local_vx, local_vy, local_omega = msg.arguments
        return  local_vx, local_vy, local_omega
    
    
    