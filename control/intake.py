import commands2
from subsystems.intake import Intake

def start_intake(intake: Intake) -> commands2.Command:
    return commands2.InstantCommand(intake.start, intake).ignoringDisable(True)

def stop_intake(intake: Intake) -> commands2.Command:
    return commands2.InstantCommand(intake.stop, intake).ignoringDisable(True)