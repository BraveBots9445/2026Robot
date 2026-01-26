from math import pi

from commands2 import (
    Command,
    RepeatCommand,
    WaitCommand,
    SequentialCommandGroup,
    InstantCommand,
)
from phoenix6 import swerve

from wpimath import applyDeadband
from wpimath.geometry import (
    Transform2d,
    Rotation2d,
    Pose2d,
    Rotation3d,
    Pose3d,
    Transform3d,
)
from wpimath.units import inchesToMeters, meters_per_second

from subsystems.vision import Vision
from subsystems.shooter import Shooter
from subsystems.turret import Turret
from subsystems.shootOnMoveCalculator import ShootOnMoveCalculator
from subsystems.fuelShootingVisualizer import FuelShootingVisualizer

from telemetry import Telemetry
from generated.tuner_constants import TunerConstants

from commands2.button import CommandXboxController

from ntcore import NetworkTableInstance
from ntcore.util import ntproperty

from wpilib import PowerDistribution, SmartDashboard

from pathplannerlib.auto import AutoBuilder, NamedCommands, PathConstraints


class RobotContainer:
    _max_speed_percent = ntproperty("MaxVelocityPercent", 1.0)
    _max_angular_rate_percent = ntproperty("MaxOmegaPercent", 1.0)

    _max_speed = TunerConstants.speed_at_12_volts
    _max_angular_rate = 0.75  # radians per second

    def __init__(self) -> None:
        self.driver_controller = CommandXboxController(0)
        # self.operator_controller = CommandXboxController(1)
        self.pdh = PowerDistribution()
        self.pdh.setSwitchableChannel(True)
        self.nettable = NetworkTableInstance.getDefault().getTable("0000DriverInfo")

        self.level = 1

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(0)  # deadband is handled in get_velocity_x/y
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
        )

        self._robot_drive = (
            swerve.requests.RobotCentric()
            .with_deadband(0)  # deadband is handled in get_velocity_x/y
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
        )

        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self.drivetrain = TunerConstants.create_drivetrain()

        self.vision = Vision(
            lambda arg1, arg2, arg3: self.drivetrain.add_vision_measurement(
                Pose2d(arg1.X(), arg1.Y(), arg1.rotation().toRotation2d()), arg2, arg3
            ),
            lambda: self.drivetrain.get_state().speeds,
            lambda: self.drivetrain.get_state().pose,
        )

        self.shooter = Shooter()
        self.shooter.setHoodAngleSetpoint(Rotation2d.fromDegrees(45))
        self.turret = Turret()
        self.shootOnMoveCalculator = ShootOnMoveCalculator(
            lambda: Pose3d(self.drivetrain.get_state().pose),
            lambda: self.drivetrain.get_state().speeds,
            Transform3d(),
            lambda v: v / inchesToMeters(2) * 60 / (2 * pi) / 0.7,
            meters_per_second(0),
            meters_per_second(20),
            Rotation2d.fromDegrees(45),
            Rotation2d.fromDegrees(90),
        )
        self.fuelShootingVisualizer = FuelShootingVisualizer(
            lambda: Pose3d(self.drivetrain.get_state().pose),
            lambda: self.drivetrain.get_state().speeds,
            self.turret.getRotation,
            self.shooter.getHoodAngle,
            lambda: self.shooter.getFlywheelVelocity(),
            inchesToMeters(2),
            Transform3d(),
        )

        self.drivetrain.register_telemetry(
            lambda telem: self._logger.telemeterize(telem)
        )

        self.set_pp_named_commands()

        self.auto_chooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData(self.auto_chooser)
        SmartDashboard.putData(self.drivetrain)

    def get_velocity_x(self) -> float:
        # x and y are swapped in wpilib vs/common convention
        # this is considered a rotation about the joystick, so forwards is negative
        x = -applyDeadband(self.driver_controller.getLeftY(), 0.05)
        return x * abs(x) * self._max_speed * self._max_speed_percent

    def get_velocity_y(self) -> float:
        # x and y are swapped in wpilib vs/common convention
        # West/left is positive in wpilib, not on controller
        y = -applyDeadband(self.driver_controller.getLeftX(), 0.05)
        return y * abs(y) * self._max_speed * self._max_speed_percent

    def get_angular_rate(self) -> float:
        t = -applyDeadband(self.driver_controller.getRightX(), 0.05)
        return t * abs(t) * self._max_angular_rate * self._max_angular_rate_percent

    def get_pathfind_constraints(self) -> PathConstraints:
        return PathConstraints(
            self._max_speed * self._max_speed_percent * 2,
            1,
            self._max_angular_rate * self._max_angular_rate_percent * 3,
            1,
        )

    def set_teleop_bindings(self) -> None:
        RepeatCommand(
            SequentialCommandGroup(
                self.fuelShootingVisualizer.launchCommand(), WaitCommand(0.1)
            ).ignoringDisable(True)
        ).ignoringDisable(True).schedule()

        def setStuff():
            setpoints = self.shootOnMoveCalculator.getSetpoints(
                Pose3d.fromFeet(182.11 / 12, 317.69 / 24, 72 / 12, Rotation3d())
            )
            if setpoints is not None:
                self.turret.setSetpoint(setpoints.turretAngle)
                self.shooter.setHoodAngleSetpoint(setpoints.hoodAngle)
                self.shooter.setFlywheelSetpoint(setpoints.flywheelRpm)

        RepeatCommand(InstantCommand(setStuff).ignoringDisable(True)).schedule()
        """driver"""
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: self._drive.with_velocity_x(self.get_velocity_x())
                .with_velocity_y(self.get_velocity_y())
                .with_rotational_rate(self.get_angular_rate())
            )
        )

        # robot oriented on Left stick push hold
        self.driver_controller.leftStick().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._robot_drive.with_velocity_x(self.get_velocity_x())
                .with_velocity_y(self.get_velocity_y())
                .with_rotational_rate(self.get_angular_rate())
            )
        )

        # slow mode and defense mode
        def half_speed():
            self._max_speed_percent /= 2
            self._max_angular_rate_percent /= 2

        def double_speed():
            self._max_speed_percent *= 2
            self._max_angular_rate_percent *= 2

        # slow mode
        self.driver_controller.leftTrigger().onTrue(InstantCommand(half_speed)).onFalse(
            InstantCommand(double_speed)
        )

        # defense mode
        self.driver_controller.rightTrigger().onTrue(
            InstantCommand(double_speed)
        ).onFalse(InstantCommand(half_speed))

        self.driver_controller.x().onTrue(self.vision.toggleEnabledCommand())

        """Operator"""
        """
        Insert code here for the secondary driver
        """
        return

        self.operator_controller.y().onTrue(
            self.turret._tmpSetSetpointCommand(Rotation2d.fromDegrees(-90))
        )

        self.operator_controller.b().onTrue(
            self.turret._tmpSetSetpointCommand(Rotation2d.fromDegrees(0))
        )

        self.operator_controller.a().onTrue(
            self.turret._tmpSetSetpointCommand(Rotation2d.fromDegrees(90))
        )
        # self.operator_controller.a().onTrue(
        #     self.shooter._tmpSetHoodAngleCommand(Rotation2d.fromDegrees(30))
        # )
        # self.operator_controller.b().onTrue(
        #     self.shooter._tmpSetHoodAngleCommand(Rotation2d.fromDegrees(0))
        # )

        # self.operator_controller.x().onTrue(self.shooter._tmpSetVelocityCommand(3000))
        # self.operator_controller.y().onTrue(self.shooter._tmpSetVelocityCommand(0))

    def set_test_bindings(self) -> None:
        # will be sysid testing for drivetrain (+others?) sometime
        self.test_remote = CommandXboxController(2)

    def set_pp_named_commands(self) -> None:
        """
        Insert code here for the pathplanner named commands
        That will be scheduled during path following
        """

    def get_auto_command(self) -> Command:
        return self.auto_chooser.getSelected()
