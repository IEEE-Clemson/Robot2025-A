import time
import smbus2
import struct
from cu_hal.auxilliary.auxilliary_hw import AuxilliaryHW
bus = smbus2.SMBus(1)

aux = AuxilliaryHW(bus)

print("Testing beacon")
aux.travel_beacon()
time.sleep(3)
aux.extend_beacon()
time.sleep(3)
aux.retract_beacon()
time.sleep(3)

# print("Testing dropper")
# print("Dropper running", aux.is_dropper_running())
# aux.extend_dropper()
# time.sleep(0.5)
# while aux.is_dropper_running():
#     time.sleep(0.1)
# print("dropper complete")