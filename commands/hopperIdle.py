from commands2 import Command

from subsystems.hopper import HopperFloor


class HopperIdle(Command):
    def __init__(self, hopper: HopperFloor):
        self._hopper = hopper
        self.addRequirements(hopper)

    def initialize(self) -> None:
        if self._hopper.hasFuel():
            self._hopper.setSetpoint(0.25)
        else:
            self._hopper.setSetpoint(0.0)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self._hopper.setSetpoint(0.0)
