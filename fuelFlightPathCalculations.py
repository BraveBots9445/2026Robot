from math import sin, cos, pi
from typing import Callable

from commands2 import Subsystem

from ntcore import StructArrayPublisher, NetworkTableInstance, NetworkTable

from wpilib import Timer

from wpimath.geometry import Rotation2d, Pose3d, Translation3d
from wpimath.units import revolutions_per_minute, meters
from wpimath.kinematics import ChassisSpeeds


class FuelPositionCalculator(Subsystem):
    _fuels: list[tuple[Pose3d, Translation3d]]
    """
    The list of fuel positions and velocities 
    """

    _getRobotVelocity: Callable[[], ChassisSpeeds]

    _getRobotPosition: Callable[[], Pose3d]

    _getTurretAngle: Callable[[], Rotation2d]

    _getHoodAngle: Callable[[], Rotation2d]

    _getShooterWheelSpeed: Callable[[], revolutions_per_minute]

    _shooterWheelRadius: meters

    _shooterTimer: Timer

    _nettable: NetworkTable

    _fuelPosePub: StructArrayPublisher

    def __init__(
        self,
        getRobotVeloctity: Callable[[], ChassisSpeeds],
        getRobotPosition: Callable[[], Pose3d],
        getTurretAngle: Callable[[], Rotation2d],
        getHoodAngle: Callable[[], Rotation2d],
        getShooterWheelSpeed: Callable[[], revolutions_per_minute],
        wheelRadius: meters,
    ):
        self._nettable = NetworkTableInstance.getDefault().getTable("00Fuel")
        self._getRobotVelocity = getRobotVeloctity
        self._getRobotPosition = getRobotPosition
        self._getTurretAngle = getTurretAngle
        self._getHoodAngle = getHoodAngle
        self._fuels = []
        self._getShooterWheelSpeed = getShooterWheelSpeed
        self._shooterWheelRadius = wheelRadius
        self._shooterTimer = Timer()
        self._fuelPosePub = self._nettable.getStructArrayTopic(
            "FuelPoses", Pose3d
        ).publish()

        self._shooterTimer.start()

    def periodic(self) -> None:
        toRemove: list[int] = []
        dt = 0.02
        for idx, (fuel, velocity) in enumerate(self._fuels):
            if fuel.translation().z < -5:
                toRemove.append(idx)
                continue
            self._fuels[idx] = (
                Pose3d(
                    Translation3d(
                        fuel.translation().x + velocity.x * dt,
                        fuel.translation().y + velocity.y * dt,
                        fuel.translation().z + velocity.z * dt,
                    ),
                    fuel.rotation(),
                ),
                Translation3d(
                    velocity.x,
                    velocity.y,
                    velocity.z - 9.81 * dt,
                ),
            )

        for removeIdx in toRemove:
            self._fuels.pop(removeIdx)
        toRemove.clear()

        self._fuelPosePub.set([fuel for (fuel, _) in self._fuels])

        if self._shooterTimer.hasElapsed(0.5):
            self.launch()
            self._shooterTimer.restart()

    def launch(self) -> None:
        robotVelocity = self._getRobotVelocity()
        robotPosition = self._getRobotPosition()
        launchSpeed = (
            self._getShooterWheelSpeed() * 2 * pi * self._shooterWheelRadius / 60
        )
        turretFacingAngle = (
            robotPosition.rotation().toRotation2d() + self._getTurretAngle()
        )
        self._fuels.append(
            (
                robotPosition,
                Translation3d(
                    robotVelocity.vx
                    + launchSpeed
                    * turretFacingAngle.cos()
                    * self._getHoodAngle().cos(),
                    robotVelocity.vy
                    + launchSpeed
                    * turretFacingAngle.sin()
                    * self._getHoodAngle().cos(),
                    launchSpeed * self._getHoodAngle().sin(),
                ),
            ),
        )
