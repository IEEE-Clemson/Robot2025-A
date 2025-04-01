import time
import smbus2
import struct
from cu_hal.auxilliary.auxilliary_hw import AuxilliaryHW
bus = smbus2.SMBus(1)

aux = AuxilliaryHW(bus)

print("Testing grabber")
aux.set_intake_speed(0)