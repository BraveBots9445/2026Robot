########## STANDARD LIBRARY IMPORTS ##########

########## WPILIB IMPORTS ##########
from commands2 import Command, RepeatCommand, SequentialCommandGroup, WaitCommand
from commands2.button import Trigger
from commands2 import cmd

from wpilib import PowerDistribution, SmartDashboard

from wpimath import applyDeadband
from wpimath.geometry import Transform2d, Rotation2d, Pose2d, Rotation3d
from wpimath.units import inchesToMeters

from ntcore import NetworkTableInstance
from ntcore.util import ntproperty


########## VENDOR (etc) IMPORTS ##########
from pathplannerlib.auto import (
    AutoBuilder,
    NamedCommands,
    PathConstraints,
    EventTrigger,
)


########## SUBSYSTEM IMPORTS ##########
from subsystems.intake import Intake
from subsystems.shooter import Shooter
from subsystems.indexer import Indexer
from subsystems.hopper import HopperFloor
from subsystems.vision import Vision

# from subsystems.vision import Vision
from telemetry import Telemetry
from generated.tuner_constants import TunerConstants

########## COMMAND IMPORTS ##########
from commands import *

########## TEAM IMPORTS ##########
from tools.CommandXboxController9445 import CommandController9445
from tools.BraveLogger import BraveLogger
from tools.rebuilt import Rebuilt, RebuiltPositions


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
        self.intake = Intake()
        self.shooter = Shooter()
        self.indexer = Indexer()
        self.hopper = HopperFloor()
        self.braveLogger = BraveLogger()
        self._logger = Telemetry(self.drivetrain.getMaxSpeed())

        self.set_pp_named_commands()

        self.vision = Vision(
            lambda pose, timestamp, stdevs: self.drivetrain.add_vision_measurement(
                Vision._pose3dToPose2d(pose), timestamp, stdevs
            ),
            lambda: self.drivetrain.get_state().speeds,
            lambda: self.drivetrain.get_state().pose,
        )

        self.drivetrain.register_telemetry(
            lambda telem: self._logger.telemeterize(telem)
        )

        self.auto_chooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData(self.auto_chooser)
        SmartDashboard.putData(self.drivetrain)

    def set_teleop_bindings(self) -> None:
        """driver"""
        # self.shooter.setDefaultCommand(ShooterStatic(self.shooter))
        self.drivetrain.setDefaultCommand(
            DriveByStick(
                self.drivetrain,
                self.driver_controller.getFRCLX,
                self.driver_controller.getFRCLY,
                self.driver_controller.getFRCRY,
                fieldCentric=True,
            )
        )

        self.shooter.setDefaultCommand(
            ShooterDefault(
                self.shooter,
                lambda: self.drivetrain.get_state(),
                False,
            )
        )

        # self.intake.setDefaultCommand(IntakeSetPosition(self.intake, 30))

        # self.hopper.setDefaultCommand(HopperIdle(self.hopper))

        # robot oriented on Left stick push hold
        self.driver_controller.leftStick().toggleOnTrue(
            DriveByStick(
                self.drivetrain,
                self.driver_controller.getFRCLX,
                self.driver_controller.getFRCLY,
                self.driver_controller.getFRCRY,
                fieldCentric=False,
            )
        )

        self.driver_controller.leftTrigger().toggleOnTrue(
            IntakeDeploy(self.intake, 0.50, False)
        )

        self.driver_controller.a().whileTrue(
            IntakeEject(self.intake).alongWith(HopperEject(self.hopper))
        )

        self.driver_controller.leftBumper().toggleOnTrue(IntakeAgitate(self.intake))

        Trigger(lambda: self.operator_controller.getFRCLX() > 0.1).whileTrue(
            RepeatCommand(
                SequentialCommandGroup(
                    self.shooter.bumpFlywheelFudgeCommand(),
                    WaitCommand(0.05),
                )
            )
        )

        Trigger(lambda: self.operator_controller.getFRCLX() < -0.1).whileTrue(
            RepeatCommand(
                SequentialCommandGroup(
                    self.shooter.dumpFlywheelFudgeCommand(), WaitCommand(0.05)
                )
            )
        )

        Trigger(lambda: self.operator_controller.getFRCRX() > 0.1).whileTrue(
            RepeatCommand(
                SequentialCommandGroup(
                    self.shooter.bumpHoodFudgeCommand(), WaitCommand(0.05)
                )
            )
        )

        Trigger(lambda: self.operator_controller.getFRCRX() < -0.1).whileTrue(
            RepeatCommand(
                SequentialCommandGroup(
                    self.shooter.dumpHoodFudgeCommand(), WaitCommand(0.05)
                )
            )
        )

        self.operator_controller.a().onTrue(IntakeStow(self.intake))
        self.operator_controller.y().whileTrue(ShooterStatic(self.shooter))

        # self.driver_controller.b().onTrue(
        #     InstantCommand(self.drivetrain.seed_field_centric())
        # )

        # Drivetrain A/B/X/Y tests
        # self.driver_controller.a().onTrue(
        #     DriveToRotation(
        #         self.drivetrain,
        #         self.driver_controller.getFRCLX,
        #         self.driver_controller.getFRCLY,
        #         self.driver_controller.getFRCRY,
        #         lambda: Rotation2d().fromDegrees(180),
        #     )
        # )
        # self.driver_controller.b().onTrue(
        #     DriveToRotation(
        #         self.drivetrain,
        #         self.driver_controller.getFRCLX,
        #         self.driver_controller.getFRCLY,
        #         self.driver_controller.getFRCRY,
        #         lambda: Rotation2d().fromDegrees(-90),
        #     )
        # )
        # self.driver_controller.x().onTrue(
        #     DriveToRotation(
        #         self.drivetrain,
        #         self.driver_controller.getFRCLX,
        #         self.driver_controller.getFRCLY,
        #         self.driver_controller.getFRCRY,
        #         lambda: Rotation2d().fromDegrees(90),
        #     )
        # )
        # self.driver_controller.y().onTrue(
        #     DriveToRotation(
        #         self.drivetrain,
        #         self.driver_controller.getFRCLX,
        #         self.driver_controller.getFRCLY,
        #         self.driver_controller.getFRCRY,
        #         lambda: Rotation2d().fromDegrees(0),
        #     )
        # )

        # # Intake A/B/X/Y tests
        # # self.driver_controller.a().onTrue(IntakeDeploy(self.intake))
        # # self.driver_controller.b().onTrue(IntakeStow(self.intake))
        # # self.driver_controller.x().whileTrue(IntakeEject(self.intake))
        # # self.driver_controller.y().whileTrue(IntakeAgitate(self.intake))

        self.driver_controller.rightTrigger().onTrue(
            DriveToRotation(
                self.drivetrain,
                self.driver_controller.getFRCLX,
                self.driver_controller.getFRCLY,
                self.driver_controller.getFRCRY,
                Rebuilt.getPosition(RebuiltPositions.Hub).toPose2d().translation,
                rotateBy=Rotation2d.fromDegrees(180),
            )
        )

        self.driver_controller.rightBumper().whileTrue(
            FeedShooter(self.indexer, self.hopper)
        )  # known good 4/3/26 5:35

        # self.shooter.setDefaultCommand(ShooterStow(self.shooter))
        # self.driver_controller.povUp().whileTrue(ShooterStatic(self.shooter))

        # self.driver_controller.leftBumper().whileTrue(ShooterStow(self.shooter))

        # self.driver_controller.start().whileTrue(IndexerForward(self.indexer))
        # self.driver_controller.back().whileTrue(IndexerReverse(self.indexer))
        # self.driver_controller.povUp().whileTrue(
        #     IndexerDejam(self.indexer, timeout=0.25)
        # )

        # self.driver_controller.povRight().whileTrue(HopperFeed(self.hopper))
        # self.driver_controller.povLeft().whileTrue(HopperEject(self.hopper))

        # # self.driver_controller.x().onTrue(self.vision.toggleEnabledCommand())

        """Operator"""
        """
        Insert code here for the secondary driver
        """

    def set_test_bindings(self) -> None:
        # will be sysid testing for drivetrain (+others?) sometime
        self.test_remote = CommandController9445(2)

        self.drivetrain.setDefaultCommand(
            DriveByStick(
                self.drivetrain,
                self.driver_controller.getFRCLX,
                self.driver_controller.getFRCLY,
                self.driver_controller.getFRCRY,
                fieldCentric=True,
            )
        )

        self.shooter.setDefaultCommand(
            ShooterTuneDistance(
                self.shooter,
                self.test_remote.getFRCLX,
                self.test_remote.getFRCRX,
                lambda: self.test_remote.rightTrigger().getAsBoolean(),
                lambda: self.drivetrain.get_state()
                .pose.translation()
                .distance(
                    Rebuilt.getPosition(RebuiltPositions.Hub).toPose2d().translation()
                ),
            )
        )

        self.test_remote.rightBumper().whileTrue(FeedShooter(self.indexer, self.hopper))

    def set_pp_named_commands(self) -> None:
        """
        Insert code here for the pathplanner named commands
        That will be scheduled during path following
        """
        NamedCommands.registerCommand(
            "ShooterStatic", ShooterStatic(self.shooter, True)
        )
        NamedCommands.registerCommand(
            "FeedShooter", FeedShooter(self.indexer, self.hopper)
        )
        NamedCommands.registerCommand(
            "ShooterFlywheelReady", ShooterFlywheelReady(self.shooter)
        )
        NamedCommands.registerCommand(
            "IntakeDeploy", IntakeDeploy(self.intake, 0.5, False)
        )
        EventTrigger("IntakeDepot").whileTrue(IntakeSetPosition(self.intake, 5.0))
        EventTrigger("IntakeDeploy").onTrue(IntakeDeploy(self.intake, 0.5, False))

    def get_auto_command(self) -> Command:
        return self.auto_chooser.getSelected()
