from commands2 import Subsystem

from cu_hal.interfaces import AuxilliaryHAL


class Safety(Subsystem):
    def __init__(self, hal: AuxilliaryHAL):
        self.hal = hal

    def is_armed(self) -> bool:
        return self.hal.is_armed()

    def is_light_on(self) -> bool:
        print("Wait for LED")
        return self.hal.is_led_on()
