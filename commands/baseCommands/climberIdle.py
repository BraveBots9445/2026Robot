from commands2 import Command

from subsystems import Climber


class ClimberIdle(Command):
    def __init__(self, climber: Climber):
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self):
        self.climber.idle()

    def isFinished(self) -> bool:
        return True
