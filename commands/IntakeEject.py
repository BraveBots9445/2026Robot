from commands2 import Command
from wpilib import Timer

from subsystems.intake import Intake


class IntakeEject(Command):
    def __init__(self, intake: Intake):
        """
        Run the intake roller in reverse for 5 seconds.

        :param intake: Intake subsystem.
        :type intake: Intake
        """
        self.intake = intake
        self._timer = Timer()
        self._startingRollerSpeed = 0.0

        self.setName("IntakeEject")
        self.addRequirements(self.intake)

    def initialize(self):
        self._timer.restart()
        self._startingRollerSpeed = self.intake.getData().rollerSetpoint
        self.intake.setPivotSetpointDegrees(0.0)
        self.intake.setRollerSetpoint(0.0)

    def execute(self):
        if self.intake.atSetpoint():
            self.intake.setRollerSetpoint(-1.0)

    def end(self, interrupted: bool):
        self.intake.setRollerSetpoint(self._startingRollerSpeed)

    def isFinished(self) -> bool:
        return self._timer.hasElapsed(5.0)
