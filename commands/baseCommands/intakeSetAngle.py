from commands2 import Command

from wpimath.units import degrees

from subsystems import Intake


class IntakeSetAngle(Command):
    def __init__(self, intake: Intake, angle: degrees):
        super().__init__()
        self.intake = intake
        self.angle = angle
        self.addRequirements(intake)

    def initialize(self):
        self.intake.setPivotSetpointDegrees(self.angle)

    def isFinished(self) -> bool:
        return abs(self.intake._pivotSetpoint.degrees() - self.angle) < 1.0
