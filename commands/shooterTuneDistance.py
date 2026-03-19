from dataclasses import dataclass

from typing import Callable

from commands2 import Command

from ntcore import NetworkTableInstance
from ntcore.util import ntproperty

from wpimath.geometry import Rotation2d
from wpimath.units import meters

from wpiutil.wpistruct import make_wpistruct

from subsystems import Shooter


@make_wpistruct
@dataclass
class ShooterSetpointsStruct:
    distanceM: float
    flywheelVelocityRPM: float
    hoodAngledeg: float


class ShooterTuneDistance(Command):
    _ntableName: str = "00ShooterTuneDistance"

    flywheelAdjustmentFactor = ntproperty(
        f"{_ntableName}/flywheelAdjustmentFactor", 30.0
    )
    hoodAngleAdjustmentFactor = ntproperty(
        f"{_ntableName}/hoodAngleAdjustmentFactor", 1.0
    )

    flywheelVelocitySetpoint = ntproperty(
        f"{_ntableName}/flywheelVelocitySetpointRPM", 0.0
    )
    hoodAngleSetpointDeg = ntproperty(f"{_ntableName}/hoodAngleSetpointDeg", 20.0)

    def __init__(
        self,
        shooter: Shooter,
        getAdjustFlywheelSpeed: Callable[[], float],
        getAdjustHoodAngle: Callable[[], float],
        getStoreSetpoints: Callable[[], bool],
        getDistance: Callable[[], meters],
    ):
        """
        A command to tune the shooter's speed and angle for a given distance.

        :param shooter: The Shooter Subsytem
        :type shooter: Shooter
        :param getAdjustFlywheelSpeed: A callable to adjust the flywheel speed by its return value * self.flywheelAdjustmentFactor (an ntproperty)
        :type getAdjustFlywheelSpeed: Callable[[], float]
        :param getAdjustHoodAngle: A callable to adjust the hood angle by its return value * self.hoodAngleAdjustmentFactor (an ntproperty)
        :type getAdjustHoodAngle: Callable[[], float]
        :param getStoreSetpoints: A callable to return whether the current setpoints should be stored (e.g. when a button is pressed)
        :type getStoreSetpoints: Callable[[], bool]
        :param getDistance: A callable to get the current distance to the target
        :type getDistance: Callable[[], meters]
        """
        self.addRequirements(shooter)

        self.nettable = NetworkTableInstance.getDefault().getTable(self._ntableName)

        self.setpointsPub = self.nettable.getStructArrayTopic(
            "Setpoints", ShooterSetpointsStruct
        ).publish()
        self.setpoints = []

        self.shooter = shooter
        self.getAdjustFlywheelSpeed = getAdjustFlywheelSpeed
        self.getAdjustHoodAngle = getAdjustHoodAngle
        self.getDistance = getDistance
        self.getStoreSetpoints = getStoreSetpoints

        self.prevLogged = False

        self.hoodAngleSetpointDeg = self.shooter._hoodMaxAngle.degrees()
        self.flywheelVelocitySetpoint = 0

    def initialize(self):
        self.hoodAngleSetpointDeg = self.shooter.getHoodAngle().degrees()
        self.flywheelVelocitySetpoint = 0

    def execute(self):
        self.flywheelVelocitySetpoint += (
            self.getAdjustFlywheelSpeed() * self.flywheelAdjustmentFactor
        )
        self.hoodAngleSetpointDeg += (
            self.getAdjustHoodAngle() * self.hoodAngleAdjustmentFactor
        )

        self.shooter.setFlywheelSetpoint(self.flywheelVelocitySetpoint)
        self.shooter.setHoodAngleSetpoint(
            Rotation2d.fromDegrees(self.hoodAngleSetpointDeg)
        )

        # the subsytems do limiting, we want to apply it here so that the setpoints we store are the actual setpoints being used, not the potentially limited ones from the subsystems
        # self.flywheelVelocitySetpoint = self.shooter.getFlywheelSetpoint()
        self.hoodAngleSetpointDeg = self.shooter.getHoodAngleSetpoint().degrees()

        storeSetpoints = self.getStoreSetpoints()
        if not self.prevLogged and storeSetpoints:
            newSetpoints = ShooterSetpointsStruct(
                distanceM=self.getDistance(),
                flywheelVelocityRPM=self.shooter.getFlywheelSetpoint(),
                hoodAngledeg=self.shooter.getHoodAngle().degrees(),  # because the hood does not always reach its setpoint, we want to record what it actually does
            )
            self.setpoints.append(newSetpoints)
            self.setpointsPub.set(self.setpoints)
        self.prevLogged = storeSetpoints

    def end(self, interrupted: bool):
        self.shooter.setFlywheelSetpoint(0.0)

    def isFinished(self) -> bool:
        return False
