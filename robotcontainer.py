########## STANDARD LIBRARY IMPORTS ##########
from math import pi

########## WPILIB IMPORTS ##########
from commands2 import (
    Command,
    RepeatCommand,
    WaitCommand,
    SequentialCommandGroup,
    InstantCommand,
    DeferredCommand,
    ParallelCommandGroup,
)
from commands2.button import CommandXboxController, Trigger

from wpimath.geometry import (
    Transform2d,
    Rotation2d,
    Transform3d,
    Pose2d,
    Translation3d,
    Rotation3d,
)
from wpimath.units import inchesToMeters

from wpilib import RobotState

from subsystems.vision import Vision
from subsystems.visualizer3d import Visualizer3D
from subsystems.BraveLogger import BraveLogger

from telemetry import Telemetry
from generated.tuner_constants import TunerConstants

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
from pathplannerlib.auto import AutoBuilder, EventTrigger, NamedCommands


########## SUBSYSTEM IMPORTS ##########
from subsystems import *

from telemetry import Telemetry
from generated.tuner_constants import TunerConstants


########## COMMAND IMPORTS ##########

from commands.stateTransitionCommands.toStow import ToStow

from commands.baseCommands.drivetrainDriveFieldOriented import (
    DrivetrainDriveFieldOriented,
)
from commands.baseCommands.drivetrainDriveRobotOriented import (
    DrivetrainDriveRobotOriented,
)
from commands.baseCommands.drivetrainSpeedHalf import DrivetrainHalfSpeed
from commands.baseCommands.drivetrainSpeedDouble import DrivetrainDoubleSpeed
from commands.baseCommands.drivetrainMoveOffset import DrivetrainMoveOffset
from commands.baseCommands.drivetrainAutoAlignTrench import DrivetrainAutoAlignTrench

from commands.baseCommands.intakeSetAngle import IntakeSetAngle
from commands.baseCommands.intakeDeploy import IntakeDeploy
from commands.baseCommands.intakeRetract import IntakeRetract
from commands.baseCommands.woahvalScore import WoahvalScore
from commands.baseCommands.woahvalStop import WoahvalStop
from commands.baseCommands.indexerScore import IndexerScore
from commands.baseCommands.indexerStop import IndexerStop
from commands.baseCommands.turretSetAngle import TurretSetAngle
from commands.baseCommands.shootOnMove import ShootOnMove
from commands.baseCommands.shootStatic import ShootStatic
from commands.baseCommands.intakeSetRollerSpeed import IntakeSetRollerSpeed
from commands.baseCommands.indexerDejam import IndexerDejam
from commands.baseCommands.indexerShoot import IndexerShoot

from commands import ShooterTuneDistance

########## TEAM IMPORTS ##########
from tools.CommandXboxController9445 import CommandController9445
from tools.rebuilt import Rebuilt, RebuiltPositions

from subsystems.zoneManager import ZoneManager


class RobotContainer:
    _max_speed_percent = ntproperty("MaxVelocityPercent", 1.0)
    _max_angular_rate_percent = ntproperty("MaxOmegaPercent", 1.0)

    def __init__(self) -> None:
        self.driver_controller = CommandController9445(0)
        self.operator_controller = CommandController9445(1)
        self.test_remote = CommandController9445(2)

        self.pdh = PowerDistribution()
        self.pdh.setSwitchableChannel(True)
        self.nettable = NetworkTableInstance.getDefault().getTable("0000DriverInfo")

        self.drivetrain = TunerConstants.create_drivetrain()
        self._logger = Telemetry(self.drivetrain.getMaxSpeed())

        self.vision = Vision(
            lambda pose, timestamp, standardDevs: self.drivetrain.add_vision_measurement(
                Pose2d(pose.X(), pose.Y(), pose.rotation().toRotation2d()),
                timestamp,
                standardDevs,
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

        self.visualizer3d = Visualizer3D(
            lambda: Transform3d(
                Translation3d(0, 0, inchesToMeters(self.climber.getPositionInches())),
                Rotation3d(0, 0, 0),
            ),
            lambda: Transform3d(
                Translation3d(), Rotation3d(0, -self.intake.getAngle().radians(), 0)
            ),
            lambda: Transform3d(),
            lambda: Transform3d(Translation3d(), Rotation3d(self.turret.getRotation())),
            self.shooter.getHoodAngle,
        )

        self.fuelShootingVisualizer = FuelShootingVisualizer(
            lambda: Pose3d(self.drivetrain.get_state().pose),
            lambda: self.drivetrain.get_state().speeds,
            self.turret.getRotation,
            self.shooter.getHoodAngle,
            lambda: self.shooter.getFlywheelVelocity(),
            inchesToMeters(2),
            self.visualizer3d.transform3dToTurret,
        )

        self.shootOnMoveCalculator = ShootOnMoveCalculator(
            lambda: Pose3d(self.drivetrain.get_state().pose),
            lambda: self.drivetrain.get_state().speeds,
            self.visualizer3d.transform3dToTurret,
            lambda omega: omega
            * inchesToMeters(2)
            * 2
            * pi
            / 60
            / self.fuelShootingVisualizer._kEnergyTransferEfficiency,
        )

        self.zoneManager = ZoneManager(self.drivetrain)

        self.braveLogger = BraveLogger()

        self.drivetrain.register_telemetry(
            lambda telem: self._logger.telemeterize(telem)
        )

        self.setPathPlannerCommands()

        self.auto_chooser = AutoBuilder.buildAutoChooser()

        self.drivetrain.reset_pose(
            Rebuilt.getPosition(
                RebuiltPositions.Hub
                + Transform3d(Translation3d(-1, 0, 0), Rotation3d())
            ).toPose2d()
        )
        RepeatCommand(
            SequentialCommandGroup(
                self.fuelShootingVisualizer.launchCommand(), WaitCommand(0.1)
            ).ignoringDisable(True)
        ).ignoringDisable(True).schedule()

        Trigger(RobotState.isEnabled).onTrue(self.zoneManager.resetZonesCommand())

        SmartDashboard.putData(self.auto_chooser)
        SmartDashboard.putData(self.shooter)
        SmartDashboard.putData(self.drivetrain)

        self.test_remote.back().onTrue(self.turret._tmpResetCommand())

        self.zoneManager.getMustStowTrigger().whileTrue(
            ToStow(
                self.climber,
                self.shooter,
                self.turret,
                self.zoneManager.getMustStowBool,
            )
        )

    def set_teleop_bindings(self) -> None:
        """driver"""
        self.drivetrain.setDefaultCommand(
            DrivetrainDriveFieldOriented(
                self.drivetrain,
                self.driver_controller.getFRCLX,
                self.driver_controller.getFRCLY,
                lambda: -self.driver_controller.getFRCRY(),
                self.drivetrain.getMaxSpeed,
                self.drivetrain.getMaxAngularRateDeg,
            )
        )
        self.zoneManager.getMustStowTrigger(1.25).whileTrue(
            DrivetrainAutoAlignTrench(
                self.drivetrain,
                self.driver_controller.getFRCLX,
                self.driver_controller.getFRCLY,
            )
        )

        # self.turret.setDefaultCommand(
        #     ShootOnMove(self.shooter, self.turret, self.shootOnMoveCalculator)
        # )
        self.shooter.setDefaultCommand(
            ShootOnMove(
                self.shooter, self.turret, self.shootOnMoveCalculator
            ).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
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

        self.driver_controller.x().onTrue(self.vision.toggleEnabledCommand())

        self.driver_controller.a().whileTrue(IntakeDeploy(self.intake)).onFalse(
            IntakeSetRollerSpeed(self.intake, 0)
        )

        """Operator"""
        self.operator_controller.rightTrigger().onTrue(
            IndexerShoot(self.indexer)
        ).onFalse(IndexerStop(self.indexer))

        self.operator_controller.leftTrigger().onTrue(
            WoahvalScore(self.woahval)
        ).onFalse(WoahvalStop(self.woahval))

        self.operator_controller.povUp().onTrue(self.shooter.bumpFudgeCommand())
        self.operator_controller.povDown().onTrue(self.shooter.dumpFudgeCommand())

        self.operator_controller.povRight().onTrue(
            self.turret.bumpManualOffsetCommand()
        )
        self.operator_controller.povLeft().onTrue(self.turret.dumpManualOffsetCommand())

    def set_test_bindings(self) -> None:
        # self.shooter.setDefaultCommand(
        #     ShooterTuneDistance(
        #         self.shooter,
        #         self.test_remote.getFRCLX,
        #         self.test_remote.getFRCRX,
        #         self.test_remote.rightTrigger().getAsBoolean,
        #         lambda: Pose3d(self.drivetrain.get_state().pose)
        #         .translation()
        #         .distance(Rebuilt.getPosition(RebuiltPositions.Hub).translation()),
        #     )
        # )

        # self.test_remote.rightTrigger().onTrue(
        #     WaitCommand(2.0).andThen(
        #         DrivetrainMoveOffset(self.drivetrain, Transform2d(0.5, 0, Rotation2d()))
        #     )
        # )

        self.test_remote.rightTrigger().onTrue(
            WoahvalScore(self.woahval).andThen(IndexerShoot(self.indexer))
        ).onFalse(WoahvalStop(self.woahval).andThen(IndexerStop(self.indexer)))

    def setPathPlannerCommands(self) -> None:
        """
        Insert code here for the pathplanner named commands
        That will be scheduled during path following
        """
        EventTrigger("IntakeDeploy").onTrue(IntakeDeploy(self.intake))
        EventTrigger("ShootOnMove").onTrue(
            ShootOnMove(self.shooter, self.turret, self.shootOnMoveCalculator)
        )
        EventTrigger("FeedShooter").onTrue(
            ParallelCommandGroup(
                IndexerShoot(self.indexer),
                WoahvalScore(self.woahval),
            )
        )
        EventTrigger("StopFeedingShooter").onTrue(
            ParallelCommandGroup(
                IndexerStop(self.indexer),
                WoahvalStop(self.woahval),
            )
        )

        # EventTrigger("IgnoreTrenchZone3s").onTrue(
        #     self.zoneManager.getIgnoreTrenchCommand(3.0)
        # )

    def getAutoCommand(self) -> Command:
        return self.auto_chooser.getSelected()
