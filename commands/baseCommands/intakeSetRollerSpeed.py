from commands2 import Command

from subsystems.intake import Intake


class IntakeSetRollerSpeed(Command):
    def __init__(self, intake: Intake, speed: float):
        """
        Sets the speed of the intake rollers.
        :param intake: The intake subsystem.
        :param speed: The speed to set the rollers to, between -1.0 and 1.0, where positive values intake and negative values outtake.
        """
        super().__init__()
        self.intake = intake
        self.speed = speed
        self.addRequirements(intake)

    def initialize(self):
        self.intake.setRollerSetpoint(self.speed)

    def isFinished(self) -> bool:
        return True
