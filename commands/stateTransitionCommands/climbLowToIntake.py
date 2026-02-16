from commands2 import SequentialCommandGroup, WaitUntilCommand, WaitCommand

from commands.baseCommands.intakeDeploy import IntakeDeploy
from commands.baseCommands.climberStow import ClimberStow


class ClimbLowToIntake(SequentialCommandGroup):
    def __init__(self, climber, intake):
        super().__init__(
            ClimberStow(climber),
            WaitUntilCommand(lambda: not climber.getHookDeployed()),
            WaitCommand(
                0.5
            ),  # the servo takes time to move, but we don't know where it is
            IntakeDeploy(intake),
            WaitUntilCommand(lambda: intake.getDeployed()),
        )
