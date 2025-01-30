from encoder import Encoder
from pi_motor import PIMotor
import utime
#import comms
from machine import Timer

motor = PIMotor(0, 3, 2, 15, 14)
voltage = 12
throttle = 1
motor.ff = 0.3
motor.p = 2.0
motor.i = 2.0
motor.max_i_acc = 1 / motor.i
motor.drive_raw(0)
utime.sleep(2)
i = 0

freq = 200
dt = 1.0 / freq
control_tim = Timer()
def control_update(timer):
    motor.update(dt)

control_tim.init(freq=freq, mode=Timer.PERIODIC, callback=control_update) #type: ignore

while True:
    utime.sleep_ms(500)
    print(motor.cur_vel)
    print(motor.out)
    print()
    # Estimate for ff in PI controller
    if motor.cur_vel > 0.1:
        print("FF value", throttle / motor.cur_vel)
        print(motor.p_term, motor.i_term, motor.ff_term, motor.__i_acc, motor.err)
        
    motor.target_velocity = 3