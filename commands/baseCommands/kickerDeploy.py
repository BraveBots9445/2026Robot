from commands2 import Command

from subsystems import Kicker


class KickerDeploy(Command):
    def __init__(self, kicker: Kicker):
        super().__init__()
        self.kicker = kicker
        self.addRequirements(kicker)

    def initialize(self):
        self.kicker.deploy()

    def isFinished(self) -> bool:
        return True
