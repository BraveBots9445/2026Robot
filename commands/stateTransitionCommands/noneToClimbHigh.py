from commands2 import SequentialCommandGroup

from commands.baseCommands.climberClimbLong import ClimberClimbLong
from commands.baseCommands.climberClimbShort import ClimberClimbShort

from commands.baseCommands.passiveHooksDeploy import PassiveHooksDeploy
from subsystems import Climber, PassiveHooks


class NoneToClimbHigh(SequentialCommandGroup):
    def __init__(self, climber: Climber, passiveHooks: PassiveHooks):
        super().__init__(
            ClimberClimbLong(climber),
            PassiveHooksDeploy(passiveHooks),
            ClimberClimbShort(climber),
            ClimberClimbShort(climber),
        )
