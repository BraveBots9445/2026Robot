from typing import Callable

from commands2 import Command

from wpimath.geometry import Pose3d, Transform3d

from subsystems import Turret


class ShootStatic(Command):
    def __init__(
        self, turret: Turret, getRobotPose: Callable[[], Pose3d], targetPose: Pose3d
    ):
        self.turret = turret
        self.getRobotPose = getRobotPose
        self.targetPose = targetPose

        self.addRequirements(self.turret)

    def execute(self):
        pose = self.getRobotPose()
        translation = pose.translation()
        targetTranslation = self.targetPose.translation()
        angleOff = (targetTranslation - translation).toTranslation2d().angle()
        self.turret.setSetpoint(angleOff - pose.rotation().toRotation2d())

    def isFinished(self) -> bool:
        return False
