""" PI controlled motor using feedback from encoder
Tested with DF Robot FIT0186
"""

from math import pi
from machine import Pin, PWM

from pid_controller import PIDController
from encoder import Encoder

def clamp(val, min_val, max_val):
    return min(max_val, max(min_val, val))


class PIMotor:
    """PI controlled motor using feedback from encoder"""

    def __init__(
        self,
        sm_id: int,
        enc_pin_a: int,
        enc_pin_b: int,
        motor_pin: int,
        direction_pin: int
    ):
        """Initializes the PI controlled motor

        Args:
            sm_id (int): ID of the state machine, must be unique
            enc_pin_a (int): GPIO pin for A channel
            enc_pin_b (int): GPIO pin for B channel
            motor_pin (int): GPIO pin for PWM
            reverse_pin (int): GPIO pin giving direction for motor_pin
        """
        self.cprad = 2800 / (2 * pi)
        "Encoder counts per rotation"
        self.invert_motor: float = False
        "Inverts the direction of the motor. Default direction is positive counter clockwise"
        self.ma_size: int = 10
        "Size of moving average filter for velocity. Accounts for samples over a ma_size * dt rnage"

        self.ff: float = 0.045
        "Percent output per rads. Helps the PID to maintain speed with less integral term [%/rads]"

        self.target_velocity: float = 0
        "Target angular velocity attempted to be reached by PI loop [rads]"
        self.cur_vel: float = 0
        "Current angular velocity of motor, filtered by moving average filter [rads]"

        self.out: float = 0

        self.__ma_buffer = [0.0 for _ in range(self.ma_size)]
        self.__ma_index = 0

        self.__encoder = Encoder(sm_id, enc_pin_a, enc_pin_b)
        self.__pwm = PWM(Pin(motor_pin))
        self.__pwm.freq(10000)
        self.__direction = Pin(direction_pin, Pin.OUT)
        # self.__pwm_r = PWM(Pin(motor_pin_r))
        # self.__pwm_r.freq(10000)

        self.__prev_count = self.__encoder.get_count()
        self.use_pi = True
        p = 0.1
        i = 0.1
        self.pid_controller = PIDController(p, i, 0)

    def update(self, dt: float):
        """Updates the control loop for the motor

        Args:
            dt (float): Time since last update [s]
        """
        self.pid_controller.setpoint = self.target_velocity
        # Update velocity
        cur_count = self.__encoder.get_count()
        if self.invert_motor:
            cur_count *= -1

        raw_vel = (cur_count - self.__prev_count) / dt
        self.__prev_count = cur_count

        self.__ma_buffer[self.__ma_index] = raw_vel
        self.__ma_index = (self.__ma_index + 1) % self.ma_size
        self.cur_vel = (float(sum(self.__ma_buffer)) / self.ma_size) / self.cprad

        self.pid_controller.update(self.cur_vel, dt)
        self.out = self.pid_controller.output + self.ff * self.target_velocity
        if self.use_pi:
            self.drive_raw(self.out)

    def drive_raw(self, percent_out: float):
        """Drives the motor with a raw percent output

        Args:
            percent_out (float): Percent output to drive the motor [-1, 1]
        """
        if self.invert_motor:
            percent_out *= -1

        deadband = 0.05
        out = 0
        # SAFETY: don't invert motor direction without stopping first
        if abs(self.cur_vel) > 0.5 and (
            (self.cur_vel > 0 and percent_out < 0)
            or (self.cur_vel < 0 and percent_out > 0)
        ):
            out = 0
        if percent_out > deadband:
            out = int(65536 * percent_out)
            self.__direction.off() # off for backwards
        elif percent_out < -deadband:
            out = int(65536 * -percent_out)
            self.__direction.on() # on for forwards
        self.__pwm.duty_u16(out)

    @property
    def p(self) -> float:
        """Proportional constant of the PID controller
        """
        return self.pid_controller.p

    @p.setter
    def p(self, val: float):
        self.pid_controller.p = val

    @property
    def i(self) -> float:
        """Integral constant of the PID controller
        """
        return self.pid_controller.i

    @i.setter
    def i(self, val: float):
        self.pid_controller.i = val
