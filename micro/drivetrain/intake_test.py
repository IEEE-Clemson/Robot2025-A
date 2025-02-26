import machine
import utime
import pi_motor

speed_pot = machine.ADC(28)
motor = pi_motor.PIMotor(0, 14, 15, 2, 3)
motor.ff = 1 / 21.8
motor.i = 1
i = 0
try:
    while True:
        motor.target_velocity = 20 * speed_pot.read_u16() / 65536
        i+=1
        if i % 50 == 0:
            print(motor.cur_vel)
        motor.update(0.01)
        utime.sleep_ms(10)
except:
    motor.drive_raw(0)