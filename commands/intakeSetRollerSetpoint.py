from commands2 import Command
from subsystems.intakeV2 import Intake

class IntakeRollerSetsetpoint(Command):
    def __init__(self, intake: Intake, rollersetpoint):
        super().__init__()
        self.addRequirements(Intake)
        self.rollersetpoint = rollersetpoint
        self.intake = intake

    def initialize(self):
        pass

    def execute(self):
        self.intake.setrollersetpoint(self.rollersetpoint)
        pass

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False
