from typing import Callable

from commands2 import Command

from ntcore import NetworkTableInstance

from wpilib import DriverStation, RobotBase
from wpimath.controller import PIDController
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.units import meters_per_second, degrees_per_second, degreesToRadians

from phoenix6 import swerve

from subsystems.ctredrivetrain import CommandSwerveDrivetrain


class DriveToRotation(Command):
    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        getX: Callable[[], float],
        getY: Callable[[], float],
        getRotation: Callable[[], float],
        getGoal: Callable[[], Rotation2d | Translation2d],
        fieldCentric: bool = True,
        flipRedAlliance: bool = True,
        rotateBy: Rotation2d = Rotation2d(),
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
        self.getGoal = getGoal
        self.flipRedAlliance = flipRedAlliance
        self.rotateBy = rotateBy

        if fieldCentric:
            self._drive = swerve.requests.FieldCentricFacingAngle()
        else:
            self._drive = swerve.requests.RobotCentricFacingAngle()

        self._drive = self._drive.with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.VELOCITY
        ).with_heading_pid(4.0, 0, 0.00)

        # self.headingPID = (
        #     PIDController(0.12, 0, 0.03)
        #     if RobotBase.isReal()
        #     else PIDController(9, 0, 0.25)
        # )
        # self.headingPID.enableContinuousInput(-180, 180)

        self.setName(
            f"DriveToRotation-{ "FieldOriented" if fieldCentric else "RobotRelative" }"
        )
        self.addRequirements(self.drivetrain)

    def execute(self):
        maxLinearSpeed = self.drivetrain.getMaxSpeed()
        # maxAngularSpeed = self.drivetrain.getMaxAngularRate()

        x = self.getX() * maxLinearSpeed
        y = self.getY() * maxLinearSpeed
        # omega = self.getRotation() * maxAngularSpeed
        target = self.getGoal()

        if isinstance(target, Translation2d):
            dtPos = self.drivetrain.get_state().pose.translation()
            newTrans = target - dtPos
            target = newTrans.angle()
            if (
                DriverStation.getAlliance() == DriverStation.Alliance.kRed
                and self.flipRedAlliance
            ):
                target = target.rotateBy(Rotation2d.fromDegrees(180))

        # omega = self.headingPID.calculate(
        #     self.drivetrain.get_state().pose.rotation().degrees(), target.degrees()
        # ) * (1 if RobotBase.isReal() else 1)

        self.drivetrain.set_control(
            self._drive.with_velocity_x(x)
            .with_velocity_y(y)
            .with_target_direction(target.rotateBy(self.rotateBy))
            # .with_rotational_rate(omega)
        )

    def isFinished(self) -> bool:
        return abs(self.getRotation()) > 0.0
