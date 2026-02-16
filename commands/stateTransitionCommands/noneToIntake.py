from commands2 import Command

from commands.baseCommands.intakeDeploy import IntakeDeploy

from subsystems import Intake


class NoneToIntake(
    IntakeDeploy
): ...  # this is just deploy the intake, but is here to allow for better readability and extensibility
