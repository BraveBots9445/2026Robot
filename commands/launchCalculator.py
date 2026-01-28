"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""
from threading import Thread
from typing import Callable
from commands2 import Command
from ntcore import NetworkTableInstance
from wpimath.geometry import Translation2d, Rotation2d, Pose2d, Translation3d, Pose3d
from wpimath.kinematics import ChassisSpeeds
from subsystems.turret import Turret
from phoenix6.swerve import SwerveDrivetrain
from tools.FuelVisualizer import Fuel

class LaunchCalculator(Command):
    def __init__(self,
                 turretPose: Callable[[], Pose2d] = lambda: Pose2d(),
                 targetPos: Translation3d = Translation3d(0, 0, 1)):
        super().__init__()
        self._getTurretPose = turretPose
        self._targetPos = targetPos
        
        _nt = NetworkTableInstance.getDefault().getTable("Calculator")
        self._log = _nt.getStructArrayTopic( "LaunchTrajectory", Pose3d ).publish()

    def initialize(self):
        print( "Start Thread:" )
        self.keepRunning = True
        self.myThread = Thread( target=self._createTrajectory )
        self.myThread.start()
        print("End of Start:")
        
    def execute(self):
        self._createTrajectory()

    def end(self, interrupted: bool):
        self.keepRunning = False
        self.fuel = []
        self._log.set( [] )
        pass

    def isFinished(self) -> bool:
        return False

    def _createTrajectory(self) -> None:
        turretPose = self._getTurretPose()

        self.fuel: list[Fuel] = []
        launchPose: list[Pose3d] = []
        incompletePath = True

        while incompletePath:
            launchPose: list[Pose3d] = []
            self.fuel.append( Fuel( Pose3d( turretPose ), Translation3d(2, 0, 7.0) ) )
            for _ in self.fuel:
                pos = _.getPosition()
                vel = _.getVelocity()
                launchPose.append( pos )

                _.periodic()

                if pos.Z() <= self._targetPos.Z() and vel.Z() < 0.0:
                    incompletePath = False
                
        self._log.set( launchPose )