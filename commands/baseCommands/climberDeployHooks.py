from commands2 import Command

from subsystems import Climber


class ClimberDeployHooks(Command):
    def __init__(self, climber: Climber):
        super().__init__()
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self):
        self.climber.deployHook()

    def isFinished(self) -> bool:
        return True
