from commands2 import Command

from subsystems.hopper import HopperFloor


class HopperFeed(Command):
    def __init__(self, hopper: HopperFloor):
        self._hopper = hopper
        self.addRequirements(hopper)

    def initialize(self) -> None:
        self._hopper.setSetpoint(1.0)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self._hopper.setSetpoint(0.0)
