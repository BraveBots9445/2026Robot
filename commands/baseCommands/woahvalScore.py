from commands2 import Command

from subsystems.woahval import Woahval


class WoahvalScore(Command):
    def __init__(self, woahval: Woahval):
        super().__init__()
        self.woahval = woahval
        self.addRequirements(woahval)

    def initialize(self):
        self.woahval.setSetpointShooting()

    def isFinished(self) -> bool:
        return True
