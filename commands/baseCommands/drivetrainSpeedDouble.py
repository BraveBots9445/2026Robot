"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""

from subsystems.ctredrivetrain import CommandSwerveDrivetrain
from commands.baseCommands.drivetrainSpeedMultiply import DrivetrainSpeedMultiply


class DrivetrainDoubleSpeed(DrivetrainSpeedMultiply):
    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
    ):
        super().__init__(drivetrain, 2.0, 2.0)
