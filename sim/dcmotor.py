import math
from numpy import sign


def clamp(x, minimum, maximum):
    return max(min(x, maximum), minimum)


class DCMotorConfig:
    def __init__(self):
        # Parameters for FIT0186
        self.stall_torque = 1.75
        self.free_load_speed = math.radians(251)
        self.rated_voltage = 12
        self.internal_moment = 0.1
        self.ppr = 2400


class DCMotorEncoderSim:
    def __init__(self, config: DCMotorConfig):
        self.config = config

        self.voltage = 0
        self.velocity = 0
        self.theta = 0
        self.external_torques = 0
        self.external_moments = 0

    def set_voltage(self, voltage: float):
        self.voltage = voltage

    def get_voltage(self) -> float:
        return self.voltage

    def get_encoder_count(self) -> int:
        return int(self.theta / self.config.ppr)

    def sim_set_velocity(self, velocity: float):
        self.velocity = velocity

    def sim_get_velocity(self) -> float:
        return self.velocity

    def sim_get_torque(self):
        # Note assumes motor is wire in brake mode
        back_emf = (
            self.config.rated_voltage * self.velocity / self.config.free_load_speed
        )
        power_proportion = (self.voltage - back_emf) / self.config.rated_voltage
        torque = power_proportion * self.config.stall_torque
        return torque

    def sim_apply_torque(self, torque):
        self.external_torques += torque
    
    def sim_apply_moment(self, moment):
        self.external_moments =moment

    def sim_step(self, dt):
        # Use implicit integration to get rid of some numerical instability
        c = self.config
        tc = c.stall_torque / (c.free_load_speed * (c.internal_moment + self.external_moments))
        k1 = self.external_torques * c.free_load_speed / c.stall_torque
        k2 = c.free_load_speed * self.voltage / c.rated_voltage

        new_vel = k1 + k2 - math.exp(-dt * tc) * (k1 + k2 - self.velocity)
        self.theta += dt * (k1 + k2) - (k1 + k2 - self.velocity - math.exp(-dt * tc) * (k1 + k2 - self.velocity))
        self.velocity = new_vel
        self.external_torques = 0
        self.external_moments = 0
