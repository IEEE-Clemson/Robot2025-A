from commands2 import Subsystem

from cu_hal.interfaces import AuxilliaryHAL

class Dumper(Subsystem):
    def __init__(self, hal: AuxilliaryHAL):
        self.hal = hal

    def dump(self):
        self.hal.extend_dropper()
