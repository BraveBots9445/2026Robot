from commands2 import Command

from subsystems.woahval import Woahval


class WoahvalIdle(Command):
    def __init__(self, woahval: Woahval):
        super().__init__()
        self.woahval = woahval
        self.addRequirements(woahval)

    def initialize(self):
        self.woahval.setSetpoint(0.1)

    def isFinished(self) -> bool:
        return True
