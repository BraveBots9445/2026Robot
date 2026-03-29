from commands2 import Command

from subsystems.intake import Intake


class IntakeSetPosition(Command):
    def __init__(self, intake: Intake, positionDegrees: float):
        """
        Set the intake pivot to a specific position in degrees.

        :param intake: Intake subsystem.
        :type intake: Intake
        :param positionDegrees: Pivot target angle in degrees.
        :type positionDegrees: float
        """
        self.intake = intake
        self.positionDegrees = positionDegrees

        self.setName("IntakeSetPosition")
        self.addRequirements(self.intake)

    def initialize(self):
        self.intake.setPivotSetpointDegrees(self.positionDegrees)

    def isFinished(self) -> bool:
        return self.intake.atSetpoint()
