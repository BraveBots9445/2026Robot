"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""

from subsystems.ctredrivetrain import CommandSwerveDrivetrain
from commands.drivetrainSpeedMultiply import DrivetrainSpeedMultiply


class DrivetrainHalfSpeed(DrivetrainSpeedMultiply):
    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
    ):
        super().__init__(drivetrain, 0.5, 0.5)
