from commands2 import SequentialCommandGroup

from commands.baseCommands.intakeSetAngle import IntakeSetAngle
from commands.baseCommands.intakeSetRollerSpeed import IntakeSetRollerSpeed

from subsystems.intake import Intake


class IntakeDeploy(SequentialCommandGroup):
    def __init__(self, intake: Intake):
        super().__init__(
            IntakeSetRollerSpeed(intake, 0.50),
            # IntakeSetRollerSpeed(intake, 1.0),
            IntakeSetAngle(intake, 0),
        )
