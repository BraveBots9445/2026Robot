from commands2 import Command

from subsystems import Climber


class ClimberClimb(Command):
    def __init__(self, climber: Climber):
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self):
        self.climber.climb()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.climber.idleMode()
