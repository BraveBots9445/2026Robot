from commands2 import Command

from subsystems.intake import Intake


class IntakeDeploy(Command):
    def __init__(self, intake: Intake, rollerSpeed: float = 1.0):
        """
        Deploy the intake and start the roller.

        :param intake: Intake subsystem.
        :type intake: Intake
        :param rollerSpeed: Roller duty cycle to apply while deployed.
        :type rollerSpeed: float
        """
        self.intake = intake
        self.rollerSpeed = rollerSpeed

        self.setName("IntakeDeploy")
        self.addRequirements(self.intake)

    def initialize(self):
        self.intake.setPivotSetpointDegrees(0.0)
        self.intake.setRollerSetpoint(self.rollerSpeed)

    def isFinished(self) -> bool:
        return self.intake.atSetpoint()
