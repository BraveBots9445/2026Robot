from commands2 import Command

from subsystems.intake import Intake


class IntakeStow(Command):
    def __init__(self, intake: Intake):
        """
        Stow the intake and stop the roller.

        :param intake: Intake subsystem.
        :type intake: Intake
        """
        self.intake = intake

        self.setName("IntakeStow")
        self.addRequirements(self.intake)

    def initialize(self):
        self.intake.setPivotSetpointDegrees(90.0)
        self.intake.setRollerSetpoint(0.0)

    def isFinished(self) -> bool:
        return self.intake.atSetpoint()
