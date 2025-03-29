import commands2
from subsystems.mover import Mover

def grab_box(mover: Mover) -> commands2.Command:
    return commands2.InstantCommand(mover.grab, mover)\
            .andThen(commands2.WaitCommand(1).ignoringDisable(True))\
            .ignoringDisable(True)

def release_box(mover: Mover) -> commands2.Command:
    return commands2.InstantCommand(mover.release, mover)\
            .andThen(commands2.WaitCommand(1).ignoringDisable(True))\
            .ignoringDisable(True)

def mover_off(mover: Mover) -> commands2.Command:
    return commands2.InstantCommand(mover.off, mover)\
            .andThen(commands2.WaitCommand(1).ignoringDisable(True))\
            .ignoringDisable(True)