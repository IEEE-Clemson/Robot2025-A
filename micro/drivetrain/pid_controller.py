from util import clamp


class PIDController:
    def __init__(self, p, i, d):
        """Initializes the PID controller with the given constants

        Args:
            p (float): Proportional constant [%/Unit]
            i (float): Integral constant [%/Unit*s]
            d (float): Derivative constant [%/Unit/s]
        """
        self.p: float = p
        "Proportional constant [%/Unit]"
        self.i: float = i
        "Integral constant [%/Unit*s]"
        self.d: float = d
        "Derivative constant [%/Unit/s]"
        self.max_i_accum: float = 1 / i
        "Maximum value of the integral accumulator [%]"
        self.max_output: float = 1
        "Maximum output value of the controller [%]"

        self.setpoint: float = 0
        "Target value for the controller [Unit]"

        self._prev_x: float = 0
        "Previous value of the feedback to the controller [Unit]"
        self._i_accum: float = 0
        "Integral accumulator of the controller [Unit * s]"

        self._output: float = 0
        "Output of the controller [%]"

    def update(self, x: float, dt: float):
        """Updates the controller with the current feedback value

        Args:
            x (float): Current feedback value [Unit]
            dt (float): Time since last update [s]
        """
        err = self.setpoint - x
        self._i_accum += err * dt
        self._i_accum = clamp(self._i_accum, -self.max_i_accum, self.max_i_accum)

        p_term = err * self.p
        i_term = self._i_accum * self.i
        d_term = (x - self._prev_x) / dt * self.d

        self._output = clamp(
            p_term + i_term + d_term, -self.max_output, self.max_output
        )

    @property
    def output(self) -> float:
        """Output of the controller"""
        return self._output
