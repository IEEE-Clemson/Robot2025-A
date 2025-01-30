from encoder import Encoder
from pi_motor import PIMotor
import utime
import comms

motor = PIMotor(0, 2, 3, 14, 15)

i = 0
while True:
    dt = 0.002
    utime.sleep_ms(2)
    if i % 1000 == 0:
        print(motor.cur_vel)
    i += 1
    motor.update(dt)
    motor.drive_raw(0.2)