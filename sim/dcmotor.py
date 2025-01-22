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
        self.internal_moment = 1
        self.ppr = 2400


class DCMotorEncoderSim:
    def __init__(self, config: DCMotorConfig):
        self.config = config

        self.voltage = 0
        self.encoder_count = 0
        self.velocity = 0

    def set_voltage(self, voltage: float):
        self.voltage = voltage

    def get_voltage(self) -> float:
        return self.voltage

    def get_encoder_count(self) -> int:
        return self.encoder_count
    
    def sim_set_velocity(self, velocity: float):
        self.velocity = velocity
    
    def sim_get_velocity(self) -> float:
        return self.velocity

    def sim_get_torque(self):
        # Note assumes motor is wire in brake mode
        back_emf = clamp(self.config.rated_voltage * self.velocity / self.config.free_load_speed, -self.config.rated_voltage, self.config.rated_voltage)
        power_proportion = (self.voltage + back_emf) / self.config.rated_voltage
        torque = power_proportion * self.config.stall_torque
        return torque
    
    def step(self, external_moment, external_torque, dt):
        self.velocity = (self.sim_get_torque() + external_torque) / (self.config.internal_moment + external_moment) * dt
        