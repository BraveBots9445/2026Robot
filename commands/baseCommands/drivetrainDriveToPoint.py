from commands2 import Command

from wpimath.geometry import Pose2d
from wpimath.controller import PIDController

from subsystems import CommandSwerveDrivetrain


class DrivetrainDriveToPoint(Command):
    def __init__(self, drivetrain: CommandSwerveDrivetrain, target: Pose2d):
        self.drivetrain = drivetrain
        self.target = target

        self.addRequirements(drivetrain)
