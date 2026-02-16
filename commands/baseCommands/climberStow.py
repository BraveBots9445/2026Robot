from commands2 import ConditionalCommand, SequentialCommandGroup

from commands.baseCommands.climberSetState import ClimberSetState

from subsystems import Climber


class ClimberStow(ConditionalCommand):
    def __init__(self, climber: Climber):
        super().__init__(
            SequentialCommandGroup(
                ClimberSetState(climber, 5, True), ClimberSetState(climber, 0, False)
            ),
            ClimberSetState(climber, 0, False),
            lambda: climber.getPositionInches() < 5 and not climber.getHookDeployed(),
        )
