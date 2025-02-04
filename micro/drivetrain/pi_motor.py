from encoder import Encoder
from math import pi
from machine import Pin, PWM


def clamp(val, min_val, max_val):
    return min(max_val, max(min_val, val))

class PIMotor:
    """PI controlled motor using feedback from encoder"""

    def __init__(
        self,
        sm_id: int,
        enc_pin_a: int,
        enc_pin_b: int,
        motor_pin_f: int,
        motor_pin_r: int,
    ):
        self.cprad = 2800 / (2 * pi)
        "Encoder counts per rotation"
        self.invert_motor: float = False
        "Inverts the direction of the motor. Default direction is positive counter clockwise"

        self.p: float = 0.1
        "Proportional coefficient for PI controller [%/rads]"
        self.i: float = 0.1
        "Integral coefficient for PI controller [%/rads*s]"
        self.max_i_acc: float = 1.0 / self.i
        "Limit to the integral term. Prevents instability [%]"
        self.ma_size: int = 10
        "Size of moving average filter for velocity. Accounts for samples over a ma_size * dt rnage"

        self.ff: float = 0.045
        "Percent output per rads. Helps the PID to maintain speed with less integral term [%/rads]"

        self.target_velocity: float = 0
        "Target angular velocity attempted to be reached by PI loop [rads]"
        self.cur_vel: float = 0
        "Current angular velocity of motor, filtered by moving average filter [rads]"

        self.__i_acc: float = 0
        "Accumulator for I term"
        self.out: float = 0

        self.__prev_count = 0
        self.__ma_buffer = [0.0 for _ in range(self.ma_size)]
        self.__ma_index = 0

        self.__encoder = Encoder(sm_id, enc_pin_a, enc_pin_b)
        self.__pwm_f = PWM(Pin(motor_pin_f))
        self.__pwm_f.freq(10000)
        self.__pwm_r = PWM(Pin(motor_pin_r)) 
        self.__pwm_r.freq(10000)


    def update(self, dt: float):
        # Update velocity
        cur_count = self.__encoder.get_count()
        if self.invert_motor:
            cur_count *= -1

        raw_vel = (cur_count - self.__prev_count) / dt
        self.__prev_count = cur_count

        self.__ma_buffer[self.__ma_index] = raw_vel
        self.__ma_index = (self.__ma_index + 1) % self.ma_size
        self.cur_vel = (float(sum(self.__ma_buffer)) / self.ma_size) / self.cprad

        err = self.target_velocity - self.cur_vel
        self.err = err
        self.__i_acc += err * dt
        self.__i_acc = clamp(self.__i_acc, -self.max_i_acc, self.max_i_acc)

        self.p_term = err * self.p
        self.i_term = self.__i_acc * self.i
        self.ff_term = self.cur_vel * self.ff

        self.out = clamp(self.p_term + self.i_term + self.ff_term, -1, 1)
        self.drive_raw(self.out)

    def drive_raw(self, percent_out: float):
        if self.invert_motor:
            percent_out *= -1



        deadband = 0.05
        out_f = 0
        out_r = 0
        # SAFETY: don't invert motor direction without stopping first
        if abs(self.cur_vel) > 0.5 and ((self.cur_vel > 0 and percent_out < 0) or (self.cur_vel < 0 and percent_out > 0)):
            out_f = 0
            out_r = 0
        if percent_out > deadband:
            out_f = int(65536 * percent_out)
        elif percent_out < -deadband:
            out_r = int(65536 * -percent_out)
        self.__pwm_f.duty_u16(out_f)
        self.__pwm_r.duty_u16(out_r)