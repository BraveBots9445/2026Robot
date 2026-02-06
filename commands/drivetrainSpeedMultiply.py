from commands2 import Command
from subsystems.ctredrivetrain import CommandSwerveDrivetrain


class DrivetrainSpeedMultiply(Command):
    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        speedMult: float,
        angularRateMult: float,
    ):
        self.drivetrain = drivetrain
        self.addRequirements(self.drivetrain)

        self.speedMult = speedMult
        self.angularRateMult = angularRateMult

    def initialize(self):
        self.drivetrain.setMaxSpeed(self.speedMult * self.drivetrain.getMaxSpeed())
        self.drivetrain.setMaxAngularRate(
            self.angularRateMult * self.drivetrain.getMaxAngularRate()
        )

    def end(self, interrupted: bool):
        self.drivetrain.setMaxSpeed(self.drivetrain.getMaxSpeed() / self.speedMult)
        self.drivetrain.setMaxAngularRate(
            self.drivetrain.getMaxAngularRate() / self.angularRateMult
        )

    def isFinished(self) -> bool:
        return True
