from commands2 import (
    SequentialCommandGroup,
    WaitUntilCommand,
)

from commands.stateTransitionCommands.noneToClimbHigh import NoneToClimbHigh

from commands.baseCommands.intakeRetract import IntakeRetract


class IntakeToClimbHigh(SequentialCommandGroup):
    def __init__(self, climber, intake):
        super().__init__(
            IntakeRetract(intake),
            WaitUntilCommand(lambda: not intake.getDeployed()),
            NoneToClimbHigh(climber),
        )
