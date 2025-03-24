import commands2
from subsystems.beacon import Beacon

def extend_beacon(beacon: Beacon) -> commands2.Command:
    return commands2.InstantCommand(beacon.extend, Beacon)\
            .andThen(commands2.WaitCommand(1).ignoringDisable(True))\
            .ignoringDisable(True)

def travel_beacon(beacon: Beacon) -> commands2.Command:
    return commands2.InstantCommand(beacon.travel, Beacon)\
            .andThen(commands2.WaitCommand(1).ignoringDisable(True))\
            .ignoringDisable(True)

def retract_beacon(beacon: Beacon) -> commands2.Command:
    return commands2.InstantCommand(beacon.retract, Beacon)\
            .andThen(commands2.WaitCommand(1).ignoringDisable(True))\
            .ignoringDisable(True)