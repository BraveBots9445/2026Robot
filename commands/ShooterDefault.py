from typing import Callable
from enum import Enum

from commands2 import SelectCommand, RepeatCommand
from phoenix6 import swerve

from wpilib import DriverStation
from wpimath.geometry import Pose3d

from subsystems.shooter import Shooter

from tools.rebuilt import Rebuilt, RebuiltPositions

from .ShooterAtHub import ShooterAtHub
from .ShooterPass import ShooterPass
from .ShooterStatic import ShooterStatic


class ShooterDefault(SelectCommand):
    def __init__(
        self,
        shooter: Shooter,
        robotState: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState],
        enableShootOnMove: bool = False,
    ):
        self._robotState = robotState

        super().__init__(
            {
                Rebuilt.Zones.HUB: ShooterAtHub(
                    shooter,
                    robotState,
                    enableShootOnMove,
                ),
                Rebuilt.Zones.PASSLEFT: ShooterPass(
                    shooter,
                    robotState,
                    ShooterPass.Target.LEFT,
                    enableShootOnMove,
                ),
                Rebuilt.Zones.PASSRIGHT: ShooterPass(
                    shooter,
                    robotState,
                    ShooterPass.Target.RIGHT,
                    enableShootOnMove,
                ),
            },
            self._getCurrentZone,
        )
        self._defaultCommand = ShooterStatic(shooter)
        self.setName("ShooterDefault-")

    def _getRobotState(self) -> swerve.SwerveDrivetrain.SwerveDriveState:
        return self._robotState()

    def _getCurrentZone(self) -> Rebuilt.Zones | None:
        return Rebuilt.getZone(self._getRobotState().pose)

    def initialize(self):
        super().initialize()
        self.__prevState = self.__getCurrentState()
        self.setName(f"ShooterDefault-{self._selectedCommand.getName()}")

    def isFinished(self) -> bool:
        _changedState = self.__hasStateChanged()
        _isFinished = super().isFinished()
        return _changedState or _isFinished

    def __hasStateChanged(self) -> bool:
        return self.__getPreviousState() != self.__getCurrentState()

    def __getPreviousState(self) -> Hashable:
        return self.__prevState

    def __getCurrentState(self) -> Hashable:
        return self._selector()
