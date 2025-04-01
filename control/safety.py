import commands2
from subsystems.mover import Mover
from subsystems.safety import Safety

def wait_for_armed(safety: Safety) -> commands2.Command:
    return commands2.WaitUntilCommand(safety.is_armed)

def wait_for_led_on(safety: Safety) -> commands2.Command:
    return commands2.WaitUntilCommand(lambda: safety.is_armed() and safety.is_light_on())