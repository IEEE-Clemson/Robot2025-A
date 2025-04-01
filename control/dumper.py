import commands2
from subsystems.dumper import Dumper


def extend_dumper(dumper: Dumper) -> commands2.Command:
    return commands2.InstantCommand(dumper.dump, dumper)\
            .andThen(commands2.WaitCommand(3).ignoringDisable(True))\
            .ignoringDisable(True)