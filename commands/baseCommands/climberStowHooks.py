from commands2 import Command

from subsystems import Climber


class ClimberStowHooks(Command):
    def __init__(self, climber: Climber):
        super().__init__()
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self):
        self.climber.retractHook()

    def isFinished(self) -> bool:
        return True
