from commands2 import SequentialCommandGroup

from commands.baseCommands.climberClimbLong import ClimberClimbLong
from commands.baseCommands.climberClimbShort import ClimberClimbShort

from subsystems import Climber


class ClimbLowToClimbHigh(SequentialCommandGroup):
    def __init__(self, climber: Climber):
        super().__init__(
            # TODO: Deploy Passive hooks here
            ClimberClimbShort(climber),
            ClimberClimbShort(climber),
        )
