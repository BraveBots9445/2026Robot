from commands2 import Command

from wpimath.units import inches

from subsystems import Climber


class ClimberSetHeight(Command):
    def __init__(self, climber: Climber, height: inches):
        super().__init__()
        self.climber = climber
        self.height = height
        self.addRequirements(climber)

    def initialize(self):
        self.climber.setHeightSetpoint(self.height)

    def isFinished(self) -> bool:
        return True
