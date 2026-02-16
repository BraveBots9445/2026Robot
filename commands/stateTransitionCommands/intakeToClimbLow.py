from commands2 import (
    SequentialCommandGroup,
    WaitUntilCommand,
)

from commands.stateTransitionCommands.noneToClimbLow import NoneToClimbLow

from commands.baseCommands.intakeRetract import IntakeRetract

from subsystems import Climber, Intake


class IntakeToClimbLow(SequentialCommandGroup):
    def __init__(self, climber: Climber, intake: Intake):
        super().__init__(
            IntakeRetract(intake),
            WaitUntilCommand(lambda: not intake.getDeployed()),
            NoneToClimbLow(climber),
        )
