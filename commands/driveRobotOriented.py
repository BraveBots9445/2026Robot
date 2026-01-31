from typing import Callable

from commands2 import Command

from phoenix6 import swerve

from subsystems.drivetrain import Drivetrain


class DriveRobotOriented(Command):
    def __init__(
        self,
        drivetrain: Drivetrain,
        getX: Callable[[], float],
        getY: Callable[[], float],
        getRotation: Callable[[], float],
    ):
        """
        Construct the DriveRobotOriented command
        Deadbands should be handled by the controller input functions

        :param drivetrain: The drivetrain subsystem object
        :type drivetrain: Drivetrain
        :param getX: Get the forwards velocity % input from the controller
        :type getX: Callable[[], float]
        :param getY: Get the sideways velocity % input from the controller
        :type getY: Callable[[], float]
        :param getRotation: Get the rotational velocity % input from the controller
        :type getRotation: Callable[[], float]
        """
        self.drivetrain = drivetrain
        self.addRequirements(self.drivetrain)
        self.getX = getX
        self.getY = getY
        self.getRotation = getRotation

        self._drive = swerve.requests.RobotCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.VELOCITY
        )

    def execute(self):
        self.drivetrain.set_control(
            self._drive.with_velocity_x(self.getX())
            .with_velocity_y(self.getY())
            .with_rotational_rate(self.getRotation())
        )
