from commands2 import SequentialCommandGroup

from commands.baseCommands.passiveHooksDeploy import PassiveHooksDeploy
from commands.baseCommands.climberClimbShort import ClimberClimbShort

from subsystems import Climber, PassiveHooks


class ClimbLowToClimbHigh(SequentialCommandGroup):
    def __init__(self, climber: Climber, passiveHooks: PassiveHooks):
        super().__init__(
            PassiveHooksDeploy(passiveHooks),
            ClimberClimbShort(climber),
            ClimberClimbShort(climber),
        )
