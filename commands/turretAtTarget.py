"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""
from typing import Callable
from commands2 import Command
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from phoenix6.swerve import SwerveDrivetrain
from subsystems.turret import Turret


class TurretAtTarget(Command):
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
        position = self.targetPosition - self.getRobotState().pose.translation()
        degrees = position.angle().degrees()
        self.turret.setPosition(degrees, True)

    def execute(self):
        pass

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return self.turret.atPosition()
