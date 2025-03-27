from commands2 import Subsystem

from cu_hal.interfaces import AuxilliaryHAL


class Mover(Subsystem):
    def __init__(self, hal: AuxilliaryHAL):
        self.hal = hal
        hal.grab_box()

    def release(self):
        self.hal.release_box()

    def grab(self):
        self.hal.grab_box()
