"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""
from typing import Callable
from commands2 import Command
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds
from subsystems.turret import Turret
from phoenix6.swerve import SwerveDrivetrain

class TurretFollowTarget(Command):
    def __init__(self,
                 turret: Turret,
                 robotState: Callable[[], SwerveDrivetrain.SwerveDriveState] = lambda: SwerveDrivetrain.SwerveDriveState(),
                 targetPos: Translation2d = Translation2d()):
        super().__init__()
        self.turret = turret
        self.targetPosition = targetPos
        self.getRobotState = robotState

        self.addRequirements(self.turret)
        # Use addRequirements() here to declare subsystem dependencies.
        # e.g. self.addRequirements(subsystem)

    def initialize(self):
        pass

    def execute(self):
        state = self.getRobotState()
        currentPosition = self.turret.getAbsolutePose().translation() # state.pose.translation()
        fieldSpeeds: ChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds( state.speeds, state.pose.rotation() )
        nextPosition = currentPosition + Translation2d( fieldSpeeds.vx * 0.02, fieldSpeeds.vy * 0.02 )
        position = self.targetPosition - nextPosition # self.getRobotState().pose.translation()
        degrees = position.angle().degrees()
        self.turret.setPosition(degrees, True)

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False
