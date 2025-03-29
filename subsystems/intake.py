from commands2 import Subsystem

from cu_hal.interfaces import AuxilliaryHAL


class Intake(Subsystem):
    def __init__(self, hal: AuxilliaryHAL):
        self.hal = hal
        hal.set_intake_speed(0)

    def start(self):
        self.hal.set_intake_speed(19.99)

    def stop(self):
        self.hal.set_intake_speed(0)

