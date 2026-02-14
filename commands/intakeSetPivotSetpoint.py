from commands2 import Command
from subsystems.intakeV2 import Intake

class IntakePivotSetsetpoint(Command):
    def __init__(self, intake: Intake, pivotsetpoint):
        super().__init__()
        self.addRequirements(Intake)
        self.pivotsetpoint = pivotsetpoint
        self.intake = intake

    def initialize(self):
        pass

    def execute(self):
        self.intake.setpivotsetpoint(self.pivotsetpoint)
        pass

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False
