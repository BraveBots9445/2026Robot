########## STANDARD LIBRARY IMPORTS ##########

########## WPILIB IMPORTS ##########
from commands2 import (
    Command,
    InstantCommand,
)
from wpilib import PowerDistribution, SmartDashboard

from wpimath import applyDeadband
from wpimath.geometry import Transform2d, Rotation2d, Pose2d, Rotation3d
from wpimath.units import inchesToMeters

from ntcore import NetworkTableInstance
from ntcore.util import ntproperty

########## VENDOR (etc) IMPORTS ##########
from pathplannerlib.auto import AutoBuilder, NamedCommands, PathConstraints

########## SUBSYSTEM IMPORTS ##########
from telemetry import Telemetry
from generated.tuner_constants import TunerConstants

########## COMMAND IMPORTS ##########
from commands import *

########## TEAM IMPORTS ##########
from tools.CommandXboxController9445 import CommandController9445


class RobotContainer:
    _max_speed_percent = ntproperty("MaxVelocityPercent", 1.0)
    _max_angular_rate_percent = ntproperty("MaxOmegaPercent", 1.0)

    def __init__(self) -> None:
        self.driver_controller = CommandController9445(0)
        self.operator_controller = CommandController9445(1)
        self.pdh = PowerDistribution()
        self.pdh.setSwitchableChannel(True)
        self.nettable = NetworkTableInstance.getDefault().getTable("0000DriverInfo")

        self.level = 1

        self.drivetrain = TunerConstants.create_drivetrain()
        self._logger = Telemetry(self.drivetrain.getMaxSpeed())

        self.drivetrain.register_telemetry(
            lambda telem: self._logger.telemeterize(telem)
        )

        self.set_pp_named_commands()

        self.auto_chooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData(self.auto_chooser)
        SmartDashboard.putData(self.drivetrain)

    def set_teleop_bindings(self) -> None:
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

        self.driver_controller.b().onTrue(
            InstantCommand(self.drivetrain.seed_field_centric)
        )

        """Operator"""
        """
        Insert code here for the secondary driver
        """

    def set_test_bindings(self) -> None:
        # will be sysid testing for drivetrain (+others?) sometime
        self.test_remote = CommandController9445(2)

    def set_pp_named_commands(self) -> None:
        """
        Insert code here for the pathplanner named commands
        That will be scheduled during path following
        """

    def get_auto_command(self) -> Command:
        return self.auto_chooser.getSelected()
