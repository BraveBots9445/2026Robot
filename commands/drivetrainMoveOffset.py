from math import pi

from commands2 import Command

from wpilib import SmartDashboard, DriverStation

from wpimath.geometry import Transform2d, Pose2d
from wpimath.controller import PIDController

from phoenix6.swerve.requests import FieldCentric

from subsystems import CommandSwerveDrivetrain


class DrivetrainMoveOffset(Command):
    def __init__(
        self, drivetrain: CommandSwerveDrivetrain, offset: Transform2d
    ) -> None:
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        self.xpid = PIDController(10.0, 0.0, 0.0)
        self.ypid = PIDController(10.0, 0.0, 0.0)
        self.tpid = PIDController(1.0, 0.0, 0.0)

        self.tpid.enableContinuousInput(-pi, pi)

        self.xpid.setTolerance(0.05)
        self.ypid.setTolerance(0.05)
        self.tpid.setTolerance(pi / 24)

        self.offset = offset

        SmartDashboard.putData("Drivetrain Move Offset xPID", self.xpid)
        SmartDashboard.putData("Drivetrain Move Offset yPID", self.ypid)
        SmartDashboard.putData("Drivetrain Move Offset tPID", self.tpid)
        self.setpoint = Pose2d()

    def initialize(self):
        self.xpid.reset()
        self.ypid.reset()
        self.tpid.reset()
        self.setpoint = self.drivetrain.get_state().pose + self.offset

    def execute(self):
        mult = 1
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            mult = -1
        currPose = self.drivetrain.get_state().pose
        x = self.xpid.calculate(currPose.X(), self.setpoint.X()) * mult
        y = self.ypid.calculate(currPose.Y(), self.setpoint.Y()) * mult
        t = self.tpid.calculate(
            currPose.rotation().radians(), self.setpoint.rotation().radians()
        )

        self.drivetrain.set_control(
            FieldCentric().with_velocity_x(x).with_velocity_y(y).with_rotational_rate(t)
        )

        print(self.setpoint, currPose)

    def isFinished(self) -> bool:
        return (
            self.xpid.atSetpoint() and self.ypid.atSetpoint() and self.tpid.atSetpoint()
        )

    def end(self, interrupted: bool):
        self.drivetrain.set_control(FieldCentric())
