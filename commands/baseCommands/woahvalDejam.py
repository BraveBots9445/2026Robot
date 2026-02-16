from commands2 import Command

from wpilib import Timer
from wpimath.units import seconds

from subsystems.woahval import Woahval


class WoahvalDejam(Command):
    def __init__(self, woahval: Woahval, time: seconds = 0.5):
        super().__init__()
        self.woahval = woahval
        self.addRequirements(woahval)
        self.timer = Timer()
        self.time = time

    def initialize(self):
        self.timer.start()
        self.woahval.setSetpoint(-0.1)

    def execute(self):
        if self.timer.hasElapsed(self.time / 2):
            self.woahval.setSetpoint(0.1)

    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.time)

    def end(self, interrupted: bool):
        self.woahval.setSetpoint(0.0)
