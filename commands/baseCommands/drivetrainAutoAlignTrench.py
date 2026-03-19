from math import hypot

from typing import Callable

from commands2 import Command

from wpilib import Timer

from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from wpimath.units import inchesToMeters, degreesToRadians

from wpilib import RobotBase, DriverStation

from phoenix6.swerve.requests import FieldCentric

from subsystems import CommandSwerveDrivetrain, Turret

from tools.rebuilt import Rebuilt


class DrivetrainAutoAlignTrench(Command):
    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        turret: Turret,
        getX: Callable[[], float],
        getY: Callable[[], float],
    ):
        super().__init__()
        self.drivetrain = drivetrain
        self.turret = turret
        self.getX = getX
        self.getY = getY

        self.addRequirements(drivetrain, turret)

        self.endTimer = Timer()

        self.yPID = (
            PIDController(9.0, 0, 0)
            if RobotBase.isSimulation()
            else PIDController(3.0, 0, 0)
        )
        self.tPID = (
            PIDController(5.0, 0, 0)
            if RobotBase.isSimulation()
            else PIDController(3.0, 0, 0)
        )
        self.tPID.enableContinuousInput(-degreesToRadians(180), degreesToRadians(180))

        self.request = FieldCentric()

    def initialize(self):
        self.endTimer.stop()
        self.endTimer.reset()

    def execute(self):
        yMult = 1
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            yMult = -1
        currPose = self.drivetrain.get_state().pose
        targetDirectionRadians = 0
        optimizationState = SwerveModuleState(1, Rotation2d.fromDegrees(0))
        optimizationState.optimize(currPose.rotation())
        targetDirectionRadians = optimizationState.angle.radians()

        robotVelRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
            self.drivetrain.get_state().speeds, currPose.rotation()
        )

        if robotVelRelative.vx > 0:
            turretAngle = Rotation2d.fromDegrees(0)
        else:
            turretAngle = Rotation2d.fromDegrees(180)

        if abs(currPose.Y() - inchesToMeters(49.84 / 2)) < abs(
            currPose.Y() - (Rebuilt.Width - inchesToMeters(49.84 / 2))
        ):
            ySetpoint = inchesToMeters(22)
        else:
            ySetpoint = Rebuilt.Width - inchesToMeters(22)

        vy = (
            self.yPID.calculate(
                currPose.Y(),
                ySetpoint,
            )
            * yMult
        )
        vt = self.tPID.calculate(
            currPose.rotation().radians(),
            targetDirectionRadians,
        )
        x = self.getX()
        self.turret.setSetpoint(turretAngle + Rotation2d(targetDirectionRadians))
        self.drivetrain.set_control(
            self.request.with_velocity_x(
                x
                * self.drivetrain.getMaxSpeed()
                # hypot(x, self.getY())
                # * self.drivetrain.getMaxSpeed()
                # * (1 if x > 0 else -1)
                # I tried to use hypot as suggested, but it is super unintuitive.
                # TODO: See what Shane thinks
            )
            .with_velocity_y(-vy)
            .with_rotational_rate((-vt) if RobotBase.isReal() else vt)
        )

    def isFinished(self) -> bool:
        if abs(self.getY()) >= 0.75:
            self.endTimer.start()
        else:
            self.endTimer.stop()

        return self.endTimer.hasElapsed(0.25)
