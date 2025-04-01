from commands2 import Subsystem

from cu_hal.interfaces import AuxilliaryHAL


class Beacon(Subsystem):
    def __init__(self, hal: AuxilliaryHAL):
        self.hal = hal
        hal.retract_beacon()

    def extend(self):
        self.hal.extend_beacon()

    def retract(self):
        self.hal.retract_beacon()

    def travel(self):
        self.hal.travel_beacon()
