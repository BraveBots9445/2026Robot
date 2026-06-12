from typing import Callable

from wpimath.units import meters_per_second, degrees_per_second

from commands2 import Command

from phoenix6 import swerve

from subsystems.ctredrivetrain import CommandSwerveDrivetrain


class DrivetrainDriveRobotOriented(Command):
    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        getX: Callable[[], float],
        getY: Callable[[], float],
        getRotation: Callable[[], float],
        getMaxSpeed: Callable[[], meters_per_second],
        getMaxAngularRate: Callable[[], degrees_per_second],
    ):
        """
        Construct the DrivetrainDriveRobotOriented command
        Deadbands should be handled by the controller input functions

        :param drivetrain: The drivetrain subsystem object
        :type drivetrain: Drivetrain
        :param getX: Get the forwards velocity % input from the controller
        :type getX: Callable[[], float]
        :param getY: Get the sideways velocity % input from the controller
        :type getY: Callable[[], float]
        :param getRotation: Get the rotational velocity % input from the controller
        :type getRotation: Callable[[], float]
        :param getMaxSpeed: A callable to get the maximum speed of the robot in meters per second
        :type getMaxSpeed: Callable[[], meters_per_second]
        :param getMaxAngularRate: A callable to get the maximum angular rate of the robot in degrees per second
        :type getMaxAngularRate: Callable[[], degrees_per_second]
        """
        self.drivetrain = drivetrain
        self.addRequirements(self.drivetrain)
        self.getX = getX
        self.getY = getY
        self.getRotation = getRotation
        self.getMaxSpeed = getMaxSpeed
        self.getMaxAngularRate = getMaxAngularRate

        self._drive = swerve.requests.RobotCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.VELOCITY
        )

    def execute(self):
        self.drivetrain.set_control(
            self._drive.with_velocity_x(self.getX() * self.getMaxSpeed())
            .with_velocity_y(self.getY() * self.getMaxSpeed())
            .with_rotational_rate(self.getRotation() * self.getMaxAngularRate())
        )
