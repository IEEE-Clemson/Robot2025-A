import time
import smbus2
import struct
from cu_hal.auxilliary.auxilliary_hw import AuxilliaryHW
bus = smbus2.SMBus(1)

aux = AuxilliaryHW(bus)

print("Testing grabber")
aux.release_box()
time.sleep(1)
aux.grab_box()
