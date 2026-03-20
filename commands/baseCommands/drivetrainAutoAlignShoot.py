from typing import Callable

from math import pi

from commands2 import Command

from wpimath.controller import PIDController
from wpimath.geometry import Pose3d

from wpilib import RobotBase

from phoenix6 import swerve
from phoenix6.swerve.requests import FieldCentric

from subsystems import CommandSwerveDrivetrain, ShootOnMoveCalculator


class DrivetrainAutoAlignShoot(Command):
    def __init__(
        self,
        shootOnMoveCalculator: ShootOnMoveCalculator,
        drivetrain: CommandSwerveDrivetrain,
        target: Pose3d,
        getX: Callable[[], float],
        getY: Callable[[], float],
    ):
        """
        A command to automatically align the robot to the target and shoot.
        assumes that the user has adjusted the target for alliance

        :param shootOnMoveCalculator: The ShootOnMoveCalculator Subsytem
        :type shootOnMoveCalculator: ShootOnMoveCalculator
        :param drivetrain: The CommandSwerveDrivetrain Subsytem
        :type drivetrain: CommandSwerveDrivetrain
        :param target: The target pose
        :type target: Pose3d
        :param getX: A function to get the x speed from the controller as duty cycle
        :type getX: Callable[[], float]
        :param getY: A function to get the y speed from the controller as duty cycle
        :type getY: Callable[[], float]
        """
        super().__init__()
        self.shootOnMoveCalculator = shootOnMoveCalculator
        self.drivetrain = drivetrain
        self.target = target
        self.getX = getX
        self.getY = getY

        self.tPID = (
            PIDController(3.0, 0, 0) if RobotBase.isReal() else PIDController(5.0, 0, 0)
        )
        self.tPID.enableContinuousInput(-pi, pi)

        self.control = FieldCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.VELOCITY
        )

        self.addRequirements(drivetrain)

    def execute(self):
        setpoints = self.shootOnMoveCalculator.getSetpoints(self.target)
        pose = self.drivetrain.get_state().pose
        angleSetpoint = setpoints.turretAngle + pose.rotation()
        tSpeed = self.tPID.calculate(pose.rotation().radians(), angleSetpoint.radians())
        self.drivetrain.set_control(
            self.control.with_velocity_x(self.getX() * self.drivetrain.getMaxSpeed())
            .with_velocity_y(self.getY() * self.drivetrain.getMaxSpeed())
            .with_rotational_rate(tSpeed)
        )
