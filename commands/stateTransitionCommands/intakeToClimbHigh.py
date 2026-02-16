from commands2 import (
    SequentialCommandGroup,
    WaitUntilCommand,
)

from commands.stateTransitionCommands.noneToClimbHigh import NoneToClimbHigh

from commands.baseCommands.intakeRetract import IntakeRetract

from subsystems import Climber, Intake, PassiveHooks


class IntakeToClimbHigh(SequentialCommandGroup):
    def __init__(self, climber: Climber, intake: Intake, passiveHooks: PassiveHooks):
        super().__init__(
            IntakeRetract(intake),
            WaitUntilCommand(lambda: not intake.getDeployed()),
            NoneToClimbHigh(climber, passiveHooks),
        )
