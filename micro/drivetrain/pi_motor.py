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
        self.cprad = 2400 / (2 * pi)
        "Encoder counts per rotation"
        self.invert_motor: float = False
        "Inverts the direction of the motor. Default direction is positive counter clockwise"

        self.p: float = 0
        "Proportional coefficient for PI controller [%/rads]"
        self.i: float = 0
        "Integral coefficient for PI controller [%/rads*s]"
        self.max_i_acc: float = 0
        "Limit to the integral term. Prevents instability [%]"
        self.ma_size: int = 10
        "Size of moving average filter for velocity. Accounts for samples over a ma_size * dt rnage"

        self.ff: float = 0
        "Percent output per rads. Helps the PID to maintain speed with less integral term [%/rads]"

        self.target_velocity = 0
        "Target angular velocity attempted to be reached by PI loop [rads]"
        self.cur_vel = 0
        "Current angular velocity of motor, filtered by moving average filter [rads]"

        self.__i_acc = 0
        "Accumulator for I term"

        self.__prev_count = 0
        self.__ma_buffer = [0.0 for _ in range(self.ma_size)]
        self.__ma_index = 0

        self.__encoder = Encoder(sm_id, enc_pin_a, enc_pin_b)
        self.__pwm_f = PWM(Pin(motor_pin_f))
        self.__pwm_f.freq(2000)
        self.__pwm_r = PWM(Pin(motor_pin_r)) 
        self.__pwm_r.freq(2000)


    def update(self, dt: float):
        # Update velocity
        cur_count = self.__encoder.get_count()
        raw_vel = (cur_count - self.__prev_count) / dt
        self.__prev_count = cur_count

        self.__ma_buffer[self.__ma_index] = raw_vel
        self.__ma_index = (self.__ma_index + 1) % self.ma_size
        self.cur_vel = sum(self.__ma_buffer) / self.ma_size

        err = self.target_velocity - self.cur_vel
        self.__i_acc += err * dt
        self.__i_acc = clamp(self.cur_vel, -self.max_i_acc, self.max_i_acc)

        p_term = err * self.p
        i_term = self.__i_acc * self.i
        ff_term = self.cur_vel * self.ff

        out = clamp(p_term + i_term + ff_term, -1, 1)
        self.drive_raw(out)

    def drive_raw(self, percent_out: float):
        deadband = 0.05
        out_f = 0
        out_r = 0
        if percent_out > deadband:
            out_f = int(65536 * percent_out)
        elif percent_out < -deadband:
            out_r = int(65536 * -percent_out)
        self.__pwm_f.duty_u16(out_f)
        self.__pwm_r.duty_u16(out_r)