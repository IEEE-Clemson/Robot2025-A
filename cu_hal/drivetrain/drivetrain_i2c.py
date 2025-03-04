from time import sleep
from typing import Tuple
import comms
from cu_hal.interfaces import DrivetrainHAL
import struct
import smbus2
bus = smbus2.SMBus(1)
V_MAX = 2.0
O_MAX = 8.0
INT16_MAX = 1 << 15

class DrivetrainI2C(DrivetrainHAL):
    """Implementation of the drivetrain HAL for physical hardware using UART communication
    """
    # def __init__(self, comm: comms.Comm):
    #     super().__init__()
    #     self.__comm = comm

    def get_local_velocity(self) -> Tuple[float, float, float]:
        byte_data = None
        for i in range(5):
            try:
                byte_data = bytes(bus.read_i2c_block_data(0x41, 6, 6))
                break
            except OSError:
                sleep(0.02)
        data = struct.unpack('<hhh', byte_data)
        # print(data)
        vx_actual = data[0] * V_MAX / INT16_MAX
        vy_actual = data[1] * V_MAX / INT16_MAX
        o_actual = data[2] * O_MAX / INT16_MAX
        return vx_actual, vy_actual, o_actual

    def set_target_wheel_velocities(self, vx: float, vy: float, omega: float) -> Tuple[float, float, float]:
        vx_raw = int(vx * INT16_MAX / V_MAX)
        vy_raw = int(vy * INT16_MAX / V_MAX)
        omega_raw = int(omega * INT16_MAX / O_MAX)


        data = struct.pack("<hhh", vx_raw, vy_raw, omega_raw)
        for i in range(5):
            try:
                bus.write_i2c_block_data(0x41, 0, data)
                break
            except OSError:
                sleep(0.02)
        
        return self.get_local_velocity()
    
    
    