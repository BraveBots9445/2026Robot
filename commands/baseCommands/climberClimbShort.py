from commands.baseCommands.climberSetState import ClimberSetState
from subsystems import Climber


class ClimberClimbShort(ClimberSetState):
    def __init__(self, climber: Climber):
        super().__init__(climber, 19, True)
