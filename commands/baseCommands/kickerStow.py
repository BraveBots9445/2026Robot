from commands2 import Command

from subsystems import Kicker


class KickerStow(Command):
    def __init__(self, kicker: Kicker):
        super().__init__()
        self.kicker = kicker
        self.addRequirements(kicker)

    def initialize(self):
        self.kicker.stow()

    def isFinished(self) -> bool:
        return True
