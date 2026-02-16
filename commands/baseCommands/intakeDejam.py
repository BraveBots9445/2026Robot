from commands2 import Command

from wpilib import Timer
from wpimath.units import seconds

from subsystems import Intake


class IntakeDejam(Command):
    def __init__(self, intake: Intake, time: seconds = 0.5):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)
        self.timer = Timer()
        self.time = time

    def initialize(self):
        self.timer.start()
        self.startingAngle = self.intake.getAngle()
        if self.startingAngle.degrees() > 85:
            self.intake.setPivotSetpointDegrees(45)
        self.intake.setRollerSetpoint(-0.5)

    def end(self, interrupted: bool):
        self.intake.setRollerSetpoint(0.0)
        self.intake.setPivotSetpoint(self.startingAngle)

    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.time)
