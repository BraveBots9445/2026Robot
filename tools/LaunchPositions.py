import numpy as np
from numpy.typing import NDArray
from wpimath.units import meters, degrees, turns_per_second

class LaunchPositions:
    distanceMeters: meters
    hoodAngleDegrees: degrees
    flywheelSpeedRps: turns_per_second

    def __init__(self, distance: meters, hoodAngle: degrees, flywheelSpeed: turns_per_second):
        self.distanceMeters = distance
        self.hoodAngleDegrees = hoodAngle
        self.flywheelSpeedRps = flywheelSpeed

    @staticmethod
    def prepareInterpolation(positions:list[LaunchPositions]) -> tuple[NDArray,NDArray,NDArray]:
        dValues = []
        hValues = []
        fValues = []

        for _ in positions:
            dValues.append( _.distanceMeters )
            hValues.append( _.hoodAngleDegrees )
            fValues.append( _.flywheelSpeedRps )

        return np.array( dValues ), np.array( hValues ), np.array( fValues )

class LaunchConstants:
    shootingPositions: list[LaunchPositions] =[
        LaunchPositions(0.0, 0.0, 0.0),
        LaunchPositions(1.0, 0.0, 0.0),
        LaunchPositions(2.0, 0.0, 0.0),
        LaunchPositions(3.0, 0.0, 0.0),
        LaunchPositions(4.0, 0.0, 0.0),
        LaunchPositions(5.0, 0.0, 0.0),
        LaunchPositions(6.0, 0.0, 0.0)
    ]

    passingPositions: list[LaunchPositions] = [
        LaunchPositions(0.0, 0.0, 0.0),
        LaunchPositions(1.0, 0.0, 0.0),
        LaunchPositions(2.0, 0.0, 0.0),
        LaunchPositions(3.0, 0.0, 0.0),
        LaunchPositions(4.0, 0.0, 0.0),
        LaunchPositions(5.0, 0.0, 0.0),
        LaunchPositions(6.0, 0.0, 0.0),
        LaunchPositions(7.0, 0.0, 0.0),
        LaunchPositions(8.0, 0.0, 0.0),
        LaunchPositions(9.0, 0.0, 0.0)
    ]