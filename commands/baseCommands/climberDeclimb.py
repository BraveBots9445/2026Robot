from commands2 import SequentialCommandGroup

from commands.baseCommands.climberSetHeight import ClimberSetHeight
from commands.baseCommands.climberStowHooks import ClimberStowHooks
from commands.baseCommands.climberSetState import ClimberSetState
from subsystems.climber import Climber


class ClimberDeclimb(SequentialCommandGroup):
    def __init__(self, climber: Climber):
        super().__init__()
        self.addCommands(
            ClimberSetHeight(climber, climber.maxHeight),
            ClimberSetState(climber, climber.minHeight, False),
        )
