"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""

from typing import Callable
import numpy as np
from commands2 import Command, Subsystem
from wpimath.geometry import Pose2d, Translation2d
from tools.LaunchPositions import LaunchPositions

shootingPositions: list[LaunchPositions] =[
    LaunchPositions(0.0, 85.0, 100.0),
    LaunchPositions(1.0, 80.0, 125.0),
    LaunchPositions(2.0, 70.0, 150.0),
    LaunchPositions(3.0, 60.0, 175.0),
    LaunchPositions(4.0, 50.0, 200.0),
    LaunchPositions(5.0, 45.0, 250.0),
    LaunchPositions(6.0, 60.0, 300.0)
]

class PrepareShooterEstimate(Command):
    def __init__(self,
                 hood: Subsystem,
                 shooter: Subsystem,
                 currentPose: Callable[[], Pose2d]):
        super().__init__()

        self.addRequirements( hood, shooter )
        self._hoodSystem = hood
        self._shooterSystem = shooter       
        self.getCurrentPose = currentPose
        
        self._distance, self._hood, self._flywheel = LaunchPositions.prepareInterpolation( shootingPositions )

    def initialize(self):
        pass

    def execute(self):
        currentXY = self.getCurrentPose().translation()
        targetXY = Translation2d( 4, 4 )
        distance = currentXY.distance( targetXY )
        
        hoodPos = np.interp( distance, self._distance, self._hood )
        self._hoodSystem.setDefaultCommand( self._hoodSystem.idle().withName(f"Hood: {hoodPos}") )
        
        flywheelSpeed = np.interp( distance, self._distance, self._flywheel )
        self._shooterSystem.setDefaultCommand( self._shooterSystem.idle().withName(f"Flywheel: {flywheelSpeed}") )
        

    def end(self, interrupted: bool):
        self._hoodSystem.setDefaultCommand( self._hoodSystem.idle() )
        self._shooterSystem.setDefaultCommand( self._shooterSystem.idle() )
        pass

    def isFinished(self) -> bool:
        return False
