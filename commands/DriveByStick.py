from typing import Callable

from commands2 import Command

from ntcore import NetworkTableInstance

from wpimath.units import meters_per_second, degrees_per_second, degreesToRadians

from phoenix6 import swerve

from subsystems.ctredrivetrain import CommandSwerveDrivetrain


class DriveByStick(Command):
    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        getX: Callable[[], float],
        getY: Callable[[], float],
        getRotation: Callable[[], float],
        fieldCentric: bool = True,
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
        :param fieldCentric: Establishes if the driving is Field Oriented or Robot Relative
        :type fieldCentric: bool
        """
        self.drivetrain = drivetrain

        self.getX = getX
        self.getY = getY
        self.getRotation = getRotation

        if fieldCentric:
            self._drive = swerve.requests.FieldCentric()
        else:
            self._drive = swerve.requests.RobotCentric()

        self._drive.with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.VELOCITY
        )

        self.setName(f"Drive-{ "FieldOriented" if fieldCentric else "RobotRelative" }")
        self.addRequirements(self.drivetrain)

    def execute(self):
        maxLinearSpeed  = self.drivetrain.getMaxSpeed()
        maxAngularSpeed = self.drivetrain.getMaxAngularRate()

        x = self.getX() * maxLinearSpeed
        y = self.getY() * maxLinearSpeed
        omega = self.getRotation() * maxAngularSpeed

        self.drivetrain.set_control(
            self._drive.with_velocity_x(x)
            .with_velocity_y(y)
            .with_rotational_rate(omega)
        )
