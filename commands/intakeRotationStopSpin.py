from commands2 import Command
from subsystems.intake import Intake

class IntakeRoationStopSpin(Command):
    def __init__(self, intake: Intake):
        super().__init__()
        self.addRequirements(intake)
        self.intake = intake

    def initialize(self):
        pass

    def execute(self):
        self.intake.set_speed(0)
        pass

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False