from commands2 import Command

from subsystems.woahval import Woahval


class WoahvalStop(Command):
    def __init__(self, woahval: Woahval):
        super().__init__()
        self.woahval = woahval
        self.addRequirements(woahval)

    def execute(self):
        self.woahval.setSetpoint(0.0)

    def isFinished(self) -> bool:
        return abs(self.woahval.getSetpoint()) <= 0.02
