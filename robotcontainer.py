########## STANDARD LIBRARY IMPORTS ##########
from math import pi

########## WPILIB IMPORTS ##########
from commands2 import (
    Command,
    RepeatCommand,
    WaitCommand,
    SequentialCommandGroup,
)
from ntcore import NetworkTableInstance
from ntcore.util import ntproperty

from wpilib import PowerDistribution, SmartDashboard

from wpimath.geometry import (
    Rotation2d,
    Pose2d,
    Pose3d,
    Transform3d,
    Transform2d,
)
from wpimath.units import inchesToMeters

########## VENDOR (etc) IMPORTS ##########
from pathplannerlib.auto import AutoBuilder, NamedCommands, PathConstraints


########## SUBSYSTEM IMPORTS ##########
from subsystems import *

from telemetry import Telemetry
from generated.tuner_constants import TunerConstants


from telemetry import Telemetry
from generated.tuner_constants import TunerConstants

########## COMMAND IMPORTS ##########
from commands.baseCommands.drivetrainDriveFieldOriented import (
    DrivetrainDriveFieldOriented,
)
from commands.baseCommands.drivetrainDriveRobotOriented import (
    DrivetrainDriveRobotOriented,
)
from commands.baseCommands.drivetrainSpeedHalf import DrivetrainHalfSpeed
from commands.baseCommands.drivetrainSpeedDouble import DrivetrainDoubleSpeed
from commands.baseCommands.drivetrainMoveOffset import DrivetrainMoveOffset

from commands import ShooterTuneDistance

########## TEAM IMPORTS ##########
from tools.CommandXboxController9445 import CommandController9445
from tools.rebuilt import Rebuilt, RebuiltPositions

from subsystems.stateManger import StateManager


class RobotContainer:
    _max_speed_percent = ntproperty("MaxVelocityPercent", 1.0)
    _max_angular_rate_percent = ntproperty("MaxOmegaPercent", 1.0)

    def __init__(self) -> None:
        self.driver_controller = CommandController9445(0)
        self.operator_controller = CommandController9445(1)
        self.pdh = PowerDistribution()
        self.pdh.setSwitchableChannel(True)
        self.nettable = NetworkTableInstance.getDefault().getTable("0000DriverInfo")

        self.drivetrain = TunerConstants.create_drivetrain()
        self._logger = Telemetry(self.drivetrain.getMaxSpeed())

        self.vision = Vision(
            lambda arg1, arg2, arg3: self.drivetrain.add_vision_measurement(
                Pose2d(arg1.X(), arg1.Y(), arg1.rotation().toRotation2d()), arg2, arg3
            ),
            lambda: self.drivetrain.get_state().speeds,
            lambda: self.drivetrain.get_state().pose,
        )

        self.shooter = Shooter()
        self.turret = Turret()
        self.intake = Intake()
        self.climber = Climber()
        self.kicker = Kicker()
        self.indexer = Indexer()
        self.woahval = Woahval()
        self.passiveHooks = PassiveHooks()
        self.shootOnMoveCalculator = ShootOnMoveCalculator(
            lambda: Pose3d(self.drivetrain.get_state().pose),
            lambda: self.drivetrain.get_state().speeds,
            Transform3d(),
            lambda v: v / inchesToMeters(2) * 60 / (2 * pi) / 0.7,
            0,
            20,
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

        self.stateManger = StateManager(
            self.drivetrain,
            self.shooter,
            self.turret,
            self.kicker,
            self.indexer,
            self.woahval,
            self.climber,
            self.intake,
            self.passiveHooks,
            self.shootOnMoveCalculator,
        )

        self.drivetrain.register_telemetry(
            lambda telem: self._logger.telemeterize(telem)
        )

        self.set_pp_named_commands()

        self.auto_chooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData(self.auto_chooser)
        SmartDashboard.putData(self.drivetrain)

    def set_teleop_bindings(self) -> None:
        RepeatCommand(
            SequentialCommandGroup(
                self.fuelShootingVisualizer.launchCommand(), WaitCommand(0.1)
            ).ignoringDisable(True)
        ).ignoringDisable(True).schedule()

        self.driver_controller.a().onTrue(
            self.intake._tmpSetPivotSetpoinntCommand(Rotation2d.fromDegrees(0))
        )
        self.driver_controller.b().onTrue(
            self.intake._tmpSetPivotSetpoinntCommand(Rotation2d.fromDegrees(30))
        )
        self.driver_controller.y().onTrue(
            self.intake._tmpSetPivotSetpoinntCommand(Rotation2d.fromDegrees(90))
        )

        """driver"""
        self.drivetrain.setDefaultCommand(
            DrivetrainDriveFieldOriented(
                self.drivetrain,
                self.driver_controller.getFRCLX,
                self.driver_controller.getFRCLY,
                self.driver_controller.getFRCRY,
                self.drivetrain.getMaxSpeed,
                self.drivetrain.getMaxAngularRateDeg,
            )
        )

        # robot oriented on Left stick push hold
        self.driver_controller.leftStick().whileTrue(
            DrivetrainDriveRobotOriented(
                self.drivetrain,
                self.driver_controller.getFRCLX,
                self.driver_controller.getFRCLY,
                self.driver_controller.getFRCRY,
                self.drivetrain.getMaxSpeed,
                self.drivetrain.getMaxAngularRateDeg,
            )
        )

        # slow mode
        self.driver_controller.leftTrigger().onTrue(
            DrivetrainHalfSpeed(self.drivetrain)
        )

        # defense mode
        self.driver_controller.rightTrigger().onTrue(
            DrivetrainDoubleSpeed(self.drivetrain)
        )

        # self.driver_controller.b().onTrue(
        #     InstantCommand(self.drivetrain.seed_field_centric)
        # )

        self.driver_controller.x().onTrue(self.vision.toggleEnabledCommand())

        """Operator"""
        """
        Insert code here for the secondary driver
        """
        self.operator_controller.leftTrigger().onTrue(self.stateManger.startIntaking())
        self.operator_controller.leftBumper().onTrue(self.stateManger.stopIntaking())

        self.operator_controller.rightTrigger().onTrue(
            self.stateManger.startClimbingLow()
        )

        self.operator_controller.a().onTrue(self.stateManger.startShooting())
        self.operator_controller.b().onTrue(self.stateManger.startAiming())

    def set_test_bindings(self) -> None:
        # will be sysid testing for drivetrain (+others?) sometime
        self.test_remote = CommandController9445(2, deadband=0.1)

        self.shooter.setDefaultCommand(
            ShooterTuneDistance(
                self.shooter,
                self.test_remote.getFRCLX,
                self.test_remote.getFRCRX,
                self.test_remote.rightTrigger().getAsBoolean,
                lambda: Pose3d(self.drivetrain.get_state().pose)
                .translation()
                .distance(Rebuilt.getPosition(RebuiltPositions.Hub).translation()),
            )
        )

        self.test_remote.rightTrigger().onTrue(
            WaitCommand(2.0).andThen(
                DrivetrainMoveOffset(
                    self.drivetrain, Transform2d(-0.5, 0, Rotation2d())
                )
            )
        )

    def set_pp_named_commands(self) -> None:
        """
        Insert code here for the pathplanner named commands
        That will be scheduled during path following
        """

    def get_auto_command(self) -> Command:
        return self.auto_chooser.getSelected()
