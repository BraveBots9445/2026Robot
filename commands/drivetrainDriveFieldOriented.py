from typing import Callable

from commands2 import Command

from ntcore import NetworkTableInstance

from wpimath.units import meters_per_second, degrees_per_second, degreesToRadians

from phoenix6 import swerve

from subsystems.drivetrain import Drivetrain


class DrivetrainDriveFieldOriented(Command):
    def __init__(
        self,
        drivetrain: Drivetrain,
        getX: Callable[[], float],
        getY: Callable[[], float],
        getRotation: Callable[[], float],
        getMaxSpeed: Callable[[], meters_per_second],
        getMaxAngularRate: Callable[[], degrees_per_second],
    ):
        """
        Construct the DrivetrainDriveFieldOriented command
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

        self._drive = swerve.requests.FieldCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.VELOCITY
        )

        self.nettable = NetworkTableInstance.getDefault().getTable(
            "00CommandDrivetrainDriveFieldOriented"
        )
        self.xPub = self.nettable.getDoubleTopic("X").publish()
        self.yPub = self.nettable.getDoubleTopic("Y").publish()
        self.omegaPub = self.nettable.getDoubleTopic("Omega").publish()

    def initialize(self):
        self.nettable = NetworkTableInstance.getDefault().getTable(
            "00CommandDrivetrainDriveFieldOriented"
        )
        self.xPub = self.nettable.getDoubleTopic("X").publish()
        self.yPub = self.nettable.getDoubleTopic("Y").publish()
        self.omegaPub = self.nettable.getDoubleTopic("Omega").publish()
        return super().initialize()

    def execute(self):
        x = self.getX() * self.getMaxSpeed()
        y = self.getY() * self.getMaxSpeed()
        omega = self.getRotation() * degreesToRadians(self.getMaxAngularRate())

        self.xPub.set(x)
        self.yPub.set(y)
        self.omegaPub.set(omega)

        self.drivetrain.set_control(
            self._drive.with_velocity_x(x)
            .with_velocity_y(y)
            .with_rotational_rate(omega)
        )

    def end(self, interrupted: bool) -> None:
        del self.nettable
