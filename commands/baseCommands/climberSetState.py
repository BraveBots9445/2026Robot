from commands2 import Command

from subsystems.climber import Climber

from wpimath.units import inches


class ClimberSetState(Command):
    def __init__(self, climber: Climber, height: inches, deployHooks: bool):
        super().__init__()
        self.climber = climber
        self.height = height
        self.deployHooks = deployHooks
        self.addRequirements(climber)

    def initialize(self):
        self.climber.setHeightSetpoint(self.height)
        if self.deployHooks:
            self.climber.deployHook()
        else:
            self.climber.retractHook()

    def isFinished(self) -> bool:
        return True
