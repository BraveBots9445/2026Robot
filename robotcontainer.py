from commands2 import (
    Command,
    InstantCommand,
)
from phoenix6 import swerve

from wpimath.geometry import Transform2d, Rotation2d
from wpimath.units import inchesToMeters

from subsystems.vision import Vision
from telemetry import Telemetry
from generated.tuner_constants import TunerConstants

from tools.BraveController import BraveController
from commands.DriveByStick import DriveByStick
from commands.DriveBrake import DriveBrake
from commands.DriveReset import DriveReset

from ntcore import NetworkTableInstance

from wpilib import PowerDistribution, SmartDashboard

from pathplannerlib.auto import AutoBuilder, NamedCommands, PathConstraints


class RobotContainer:
    def __init__(self) -> None:
        self.driver_controller = BraveController(0)
        self.operator_controller = BraveController(1)
        self.pdh = PowerDistribution()
        self.pdh.setSwitchableChannel(True)
        self.nettable = NetworkTableInstance.getDefault().getTable("0000DriverInfo")

        self.level = 1

        self._logger = Telemetry(TunerConstants.speed_at_12_volts)

        self.drivetrain = TunerConstants.create_drivetrain()
        self.drivetrain.set_max_speed(TunerConstants.speed_at_12_volts, 0.75)

        self.vision = Vision(
            self.drivetrain.add_vision_measurement,
            lambda: self.drivetrain.get_state().pose,
            lambda: self.drivetrain.get_state().speeds,
        )

        self.drivetrain.register_telemetry(
            lambda telem: self._logger.telemeterize(telem)
        )

        self.set_pp_named_commands()

        self.auto_chooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData(self.auto_chooser)
        SmartDashboard.putData(self.drivetrain)

    def get_pathfind_constraints(self) -> PathConstraints:
        linear, angular = self.drivetrain.get_allowed_speeds()
        return PathConstraints(
            linear * 2,
            1,
            angular * 3,
            1,
        )

    def set_teleop_bindings(self) -> None:
        """driver"""
        # Field-centric drive as default
        self.drivetrain.setDefaultCommand(
            DriveByStick(
                self.drivetrain,
                self.driver_controller.getLeftX,
                self.driver_controller.getLeftY,
                self.driver_controller.getRightY,
                field_centric=True,
            )
        )

        # Robot-centric mode when left stick is pressed
        self.driver_controller.leftStick().whileTrue(
            DriveByStick(
                self.drivetrain,
                self.driver_controller.getLeftX,
                self.driver_controller.getLeftY,
                self.driver_controller.getRightY,
                field_centric=False,
            )
        )

        # Toggle slow mode (half speed)
        self.driver_controller.leftTrigger().onTrue(
            InstantCommand(self.drivetrain.toggle_halfspeed)
        )

        # Brake - lock wheels in X-pattern (while held)
        self.driver_controller.a().whileTrue(
            DriveBrake(self.drivetrain)
        )

        # Reset - point all wheels forward (while held)
        self.driver_controller.b().whileTrue(
            DriveReset(self.drivetrain)
        )

        self.driver_controller.x().onTrue(
            self.vision.toggle_vision_measurements_command()
        )

        """Operator"""
        """
        Insert code here for the secondary driver
        """

    def set_test_bindings(self) -> None:
        # will be sysid testing for drivetrain (+others?) sometime
        self.test_remote = BraveController(2)

    def set_pp_named_commands(self) -> None:
        """
        Insert code here for the pathplanner named commands
        That will be scheduled during path following
        """

    def get_auto_command(self) -> Command:
        return self.auto_chooser.getSelected()
