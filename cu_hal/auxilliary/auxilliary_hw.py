from time import sleep
from cu_hal.interfaces import AuxilliaryHAL
import smbus2
import struct

ADDR = 0x24 
MAX_INTAKE_SPEED = 14.0
INT16_MAX = 1 << 15

class AuxilliaryHW(AuxilliaryHAL):
    """ I2C implementation of auxilliary functions
    """

    def __init__(self, bus: smbus2.SMBus):
        self.bus = bus

    def _attempt_write(self, reg, data):
        for _ in range(5):
            try:
                self.bus.write_i2c_block_data(ADDR, reg, data)
                break
            except OSError:
                sleep(0.02)

    def _attempt_read(self, reg, size):
        byte_data = None
        for _ in range(5):
            try:
                byte_data = bytes(self.bus.read_i2c_block_data(ADDR, reg, size))
                break
            except OSError:
                sleep(0.02)
        return byte_data

    def idle_box(self):
        self._attempt_write(0x9, bytes([1])) # TODO: Implement idle in firmware

    def grab_box(self):
        self._attempt_write(0x9, bytes([0]))

    def release_box(self):
        self._attempt_write(0x9, bytes([1]))

    def travel_beacon(self):
        self._attempt_write(0x8, bytes([1]))

    def extend_beacon(self):
        self._attempt_write(0x8, bytes([2]))

    def retract_beacon(self):
        self._attempt_write(0x8, bytes([0]))

    def extend_dropper(self):
        self._attempt_write(0x6, bytes([0xAA]))

    def is_dropper_running(self) -> bool:
        data = self._attempt_read(0x7, 1)[0]
        print(data)
        return int(data) == 1

    def set_intake_speed(self, speed: float):
        self._attempt_write(0x2, struct.pack("<h", int(speed / MAX_INTAKE_SPEED * INT16_MAX)))

    def get_intake_speed(self) -> float:
        data = struct.unpack("<h", self._attempt_read(0x7, 1))
        return data * MAX_INTAKE_SPEED / INT16_MAX

    def is_armed(self) -> bool:
        return int(self._attempt_read(0x0, 1)[0]) == 1

    def is_led_on(self) -> bool:
        return int(self._attempt_read(0x1, 1)[0]) == 1
