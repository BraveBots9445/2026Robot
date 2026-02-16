from commands2 import SequentialCommandGroup

from commands.baseCommands.intakeSetAngle import IntakeSetAngle
from commands.baseCommands.intakeSetRollerSpeed import IntakeSetRollerSpeed

from subsystems.intake import Intake


class IntakeRetract(SequentialCommandGroup):
    def __init__(self, intake: Intake):
        super().__init__(
            IntakeSetRollerSpeed(intake, 0.1),
            IntakeSetAngle(intake, 90),
            IntakeSetRollerSpeed(intake, 0.0),
        )
