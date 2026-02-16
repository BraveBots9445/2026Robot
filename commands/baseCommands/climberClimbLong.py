from commands.baseCommands.climberSetState import ClimberSetState
from subsystems import Climber


class ClimberClimbLong(ClimberSetState):
    def __init__(self, climber: Climber):
        super().__init__(climber, 28, True)
