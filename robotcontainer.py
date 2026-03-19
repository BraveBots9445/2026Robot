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
    Subsystem,
    cmd,
)
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from wpimath.geometry import (
    Transform2d,
    Rotation2d,
    Transform3d,
    Pose2d,
    Translation3d,
    Rotation3d,
)
from wpimath.units import inchesToMeters
from wpimath.kinematics import ChassisSpeeds

from wpilib import RobotState, RobotBase

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
from commands.baseCommands.shooterShootOnMove import ShooterShootOnMove
from commands.baseCommands.turretShootOnMove import TurretShootOnMove
from commands.baseCommands.shootStatic import ShootStatic
from commands.baseCommands.intakeSetRollerSpeed import IntakeSetRollerSpeed
from commands.baseCommands.indexerDejam import IndexerDejam
from commands.baseCommands.indexerShoot import IndexerShoot
from commands.baseCommands.turretManual import TurretManual
from commands.baseCommands.turretResetManualOffset import TurretResetManualOffset
from commands.baseCommands.shooterSetFlywheelVelocity import ShooterSetFlywheelVelocity
from commands.baseCommands.shooterShootOrPass import ShooterShootOrPass
from commands.baseCommands.turretShootOrPass import TurretShootOrPass
from commands.baseCommands.climberClimb import ClimberClimb
from commands.baseCommands.climberDeploy import ClimberDeploy
from commands.baseCommands.climberIdle import ClimberIdle
from commands.baseCommands.shooterStowHood import ShooterStowHood
from commands.baseCommands.woahvalDejam import WoahvalDejam
from commands.baseCommands.indexerDejam import IndexerDejam
from commands.baseCommands.shooterStowHood import ShooterStowHood


from commands import ShooterTuneDistance

########## TEAM IMPORTS ##########
from tools.ButtonBoard import ButtonBoard
from tools.CommandXboxController9445 import CommandController9445
from tools.rebuilt import Rebuilt, RebuiltPositions

from subsystems.zoneManager import ZoneManager


class RobotContainer:
    _max_speed_percent = ntproperty("MaxVelocityPercent", 1.0)
    _max_angular_rate_percent = ntproperty("MaxOmegaPercent", 1.0)
    _localizationAutonomyEnabled = ntproperty("LocalizationAutonomyEnabled", True)

    def __init__(self) -> None:
        self.driver_controller = CommandController9445(0)
        self.operator_controller = CommandController9445(1)
        self.button_board = ButtonBoard(2)
        self.test_remote = CommandController9445(3)

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

        self.turret = Turret()
        self.shooter = Shooter()
        self.intake = Intake()
        self.climber = Climber()
        self.indexer = Indexer()
        self.woahval = Woahval()
        # self.passiveHooks = PassiveHooks()

        self.visualizer3d = Visualizer3D(
            lambda: Transform3d(
                Translation3d(0, 0, 0),
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
            self.shooter.getHoodAngleSetpoint,
            self.shooter.getFlywheelVelocity,
            inchesToMeters(2),
            self.visualizer3d.transform3dToTurret,
        )

        self.shootOnMoveCalculator = ShootOnMoveCalculator(
            lambda: Pose3d(self.drivetrain.get_state().pose),
            lambda: ChassisSpeeds.fromRobotRelativeSpeeds(
                (state := self.drivetrain.get_state()).speeds, state.pose.rotation()
            ),
            lambda omega: omega * inchesToMeters(2) * 2 * pi / 60,
            # self.visualizer3d.transform3dToTurret,
            # / self.fuelShootingVisualizer._kEnergyTransferEfficiency,
        )

        self.zoneManager = ZoneManager(self.drivetrain)

        self.timerManager = TimeManager()

        self.braveLogger = BraveLogger()

        self.drivetrain.register_telemetry(
            lambda telem: self._logger.telemeterize(telem)
        )

        self.setPathPlannerCommands()

        self.auto_chooser = AutoBuilder.buildAutoChooser()

        # self.drivetrain.reset_pose(
        #     Rebuilt.getPosition(
        #         RebuiltPositions.Hub
        #         + Transform3d(
        #             Translation3d(-1, 0, 0), Rotation3d.fromDegrees(0, 0, -90)
        #         )
        #     ).toPose2d()
        # )
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

    def toggleLocalizationAutonomyCommand(self) -> Command:
        def update():
            self._localizationAutonomyEnabled = not self._localizationAutonomyEnabled
            SmartDashboard.putBoolean(
                "LocalizationAutonomyEnabled", self._localizationAutonomyEnabled
            )

        return InstantCommand(update)

    def set_teleop_bindings(self) -> None:
        """driver"""
        self.drivetrain.setDefaultCommand(
            DrivetrainDriveFieldOriented(
                self.drivetrain,
                lambda: self.driver_controller.getFRCLX(),
                lambda: self.driver_controller.getFRCLY(),
                lambda: -self.driver_controller.getFRCRY(),
                self.drivetrain.getMaxSpeed,
                self.drivetrain.getMaxAngularRateDeg,
            )
        )

        self.zoneManager.getMustStowTrigger().whileTrue(
            ShooterStowHood(
                self.shooter,
            ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        )

        self.zoneManager.getMustStowTrigger(1.25).whileTrue(
            DrivetrainAutoAlignTrench(
                self.drivetrain,
                self.turret,
                self.driver_controller.getFRCLX,
                self.driver_controller.getFRCLY,
            )
        )

        self.timerManager.startTeleop()

        self.shooter.setDefaultCommand(
            ShooterShootOrPass(
                self.shooter, self.shootOnMoveCalculator, self.zoneManager
            ).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        )

        self.turret.setDefaultCommand(
            TurretShootOrPass(self.turret, self.shootOnMoveCalculator, self.zoneManager)
        )

        self.climber.setDefaultCommand(ClimberIdle(self.climber))

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

        # # slow mode
        # self.driver_controller.leftTrigger().onTrue(
        #     DrivetrainHalfSpeed(self.drivetrain)
        # )

        # # defense mode
        # self.driver_controller.rightTrigger().onTrue(
        #     DrivetrainDoubleSpeed(self.drivetrain)
        # )

        # self.driver_controller.x().onTrue(self.vision.toggleEnabledCommand())

        self.button_board.getForward().whileTrue(ClimberDeploy(self.climber))
        self.button_board.getReverse().whileTrue(ClimberClimb(self.climber))

        self.driver_controller.rightTrigger().whileTrue(
            IntakeDeploy(self.intake)
        ).onFalse(IntakeSetRollerSpeed(self.intake, 0))

        self.driver_controller.b().onTrue(
            InstantCommand(self.drivetrain.seed_field_centric)
        )

        """Operator"""
        # self.driver_controller.rightTrigger().onTrue(
        #     IndexerShoot(self.indexer)
        # ).onFalse(IndexerStop(self.indexer))

        # self.driver_controller.leftTrigger().onTrue(WoahvalScore(self.woahval)).onFalse(
        #     WoahvalStop(self.woahval)
        # )
        # self.operator_controller.a().onTrue(
        self.operator_controller.a().onTrue(
            IntakeSetRollerSpeed(self.intake, 0.4).andThen(
                RepeatCommand(
                    IntakeSetAngle(self.intake, 40)
                    .andThen(WaitCommand(0.5))
                    .andThen(
                        IntakeSetAngle(self.intake, 75),
                    )
                )
            )
        ).onFalse(
            IntakeSetAngle(self.intake, 0).andThen(IntakeSetRollerSpeed(self.intake, 0))
        )

        self.operator_controller.rightBumper().onTrue(
            self.turret.dumpManualOffsetCommand()
        )
        self.operator_controller.leftBumper().onTrue(
            self.turret.bumpManualOffsetCommand()
        )

        Trigger(lambda: self.operator_controller.getFRCRX() > 0.25).whileTrue(
            RepeatCommand(
                SequentialCommandGroup(
                    self.shooter.bumpFlywheelFudgeCommand(), WaitCommand(0.1)
                )
            )
        )
        Trigger(lambda: self.operator_controller.getFRCRX() < -0.25).whileTrue(
            RepeatCommand(
                SequentialCommandGroup(
                    self.shooter.dumpFlywheelFudgeCommand(), WaitCommand(0.1)
                )
            )
        )

        Trigger(lambda: self.operator_controller.getFRCLX() > 0.25).whileTrue(
            RepeatCommand(
                SequentialCommandGroup(
                    self.shooter.dumpHoodFudgeCommand(), WaitCommand(0.1)
                )
            )
        )
        Trigger(lambda: self.operator_controller.getFRCLX() < -0.25).whileTrue(
            RepeatCommand(
                SequentialCommandGroup(
                    self.shooter.bumpHoodFudgeCommand(), WaitCommand(0.1)
                )
            )
        )

        self.operator_controller.povUp().onTrue(self.turret._tmpResetCommand())

        # self.operator_controller.rightTrigger().onTrue(
        self.operator_controller.rightTrigger().onTrue(
            WoahvalScore(self.woahval).andThen(IndexerShoot(self.indexer))
        ).onFalse(WoahvalStop(self.woahval).andThen(IndexerStop(self.indexer)))

        self.operator_controller.y().and_(
            lambda: (not self.zoneManager.getInAllianceZoneBool())
            or (not self._localizationAutonomyEnabled)
        ).onTrue(
            WoahvalScore(self.woahval).andThen(IndexerShoot(self.indexer))
        ).onFalse(
            WoahvalStop(self.woahval).andThen(IndexerStop(self.indexer))
        )  # pass

        self.operator_controller.povRight().whileTrue(
            RepeatCommand(
                ParallelCommandGroup(
                    WoahvalDejam(self.woahval, 0.1), IndexerDejam(self.indexer, 0.1)
                )
            )
        ).onFalse(
            SequentialCommandGroup(WoahvalStop(self.woahval), IndexerStop(self.indexer))
        )

        self.operator_controller.x().whileTrue(
            ShooterStowHood(self.shooter).withInterruptBehavior(
                Command.InterruptionBehavior.kCancelIncoming
            )
        )

        # self.zoneManager.getInAllianceZoneTrigger().and_(
        #     lambda: (
        #         self.timerManager.isHubActive()
        #         and self.turret.atSetpoint()
        #         and self.shooter.atFlywheelSetpoint()
        #         and self.shooter.atHoodSetpoint()
        #         and self.zoneManager.getInAllianceZoneBool()
        #         and not self.button_board.getButton(4, 1).getAsBoolean()
        #     )
        #     and self._localizationAutonomyEnabled
        # ).whileTrue(
        #     WoahvalScore(self.woahval).andThen(IndexerShoot(self.indexer))
        # ).onFalse(
        #     WoahvalStop(self.woahval).andThen(IndexerStop(self.indexer))
        # )

        self.button_board.getButton(2, 1).onTrue(
            self.toggleLocalizationAutonomyCommand()
        )

    def set_test_bindings(self) -> None:
        # self.drivetrain.reset_pose(
        #     Rebuilt.getPosition(
        #         RebuiltPositions.Hub
        #         + Transform3d(
        #             Translation3d(-1, 0, 0), Rotation3d.fromDegrees(0, 0, -90)
        #         )
        #     ).toPose2d()
        # )

        self.test_remote.rightBumper().onTrue(self.turret.dumpManualOffsetCommand())
        self.test_remote.leftBumper().onTrue(self.turret.bumpManualOffsetCommand())

        self.drivetrain.setDefaultCommand(
            DrivetrainDriveFieldOriented(
                self.drivetrain,
                lambda: self.driver_controller.getFRCLX(),
                lambda: self.driver_controller.getFRCLY(),
                lambda: -self.driver_controller.getFRCRY(),
                self.drivetrain.getMaxSpeed,
                self.drivetrain.getMaxAngularRateDeg,
            )
        )

        self.shooter.removeDefaultCommand()
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

        self.driver_controller.rightTrigger().whileTrue(
            IntakeDeploy(self.intake)
        ).onFalse(IntakeSetRollerSpeed(self.intake, 0))
        # self.test_remote.rightTrigger().onTrue(
        #     WaitCommand(2.0).andThen(
        #         DrivetrainMoveOffset(
        #             self.drivetrain, Transform2d(-0.5, 0, Rotation2d())
        #         )
        #     )
        # )

        self.test_remote.leftTrigger().onTrue(
            WoahvalScore(self.woahval).andThen(IndexerShoot(self.indexer))
        ).onFalse(WoahvalStop(self.woahval).andThen(IndexerStop(self.indexer)))

        # self.test_remote.a().onTrue(IntakeSetAngle(self.intake, 0))
        self.test_remote.b().onTrue(
            IntakeSetAngle(self.intake, 30).andThen(
                IntakeSetRollerSpeed(self.intake, 0.1)
            )
        ).onFalse(
            IntakeSetAngle(self.intake, 0).andThen(IntakeSetRollerSpeed(self.intake, 0))
        )
        # self.test_remote.y().onTrue(IntakeSetAngle(self.intake, 90))

    def setPathPlannerCommands(self) -> None:
        """
        Insert code here for the pathplanner named commands
        That will be scheduled during path following
        """
        NamedCommands.registerCommand(
            "Flywheel4000rpm", ShooterSetFlywheelVelocity(self.shooter, 4000)
        )
        NamedCommands.registerCommand("IntakeDeploy", IntakeDeploy(self.intake))
        NamedCommands.registerCommand(
            "ShootOnMove",
            ShooterShootOnMove(self.shooter, self.shootOnMoveCalculator).alongWith(
                TurretShootOnMove(self.turret, self.shootOnMoveCalculator)
            ),
        )

        NamedCommands.registerCommand(
            "FeedShooter10s",
            SequentialCommandGroup(
                WaitCommand(0.1),
                WaitCommand(7).until(
                    lambda: (
                        self.shooter.atFlywheelSetpoint()
                        and self.shooter.atHoodSetpoint()
                        and self.turret.atSetpoint()
                    )
                    or RobotBase.isSimulation()
                ),
                RepeatCommand(
                    ParallelCommandGroup(
                        IndexerShoot(self.indexer),
                        WoahvalScore(self.woahval),
                    )
                ).withTimeout(10),
            ),
        )

        NamedCommands.registerCommand(
            "StowHood",
            ShooterStowHood(
                self.shooter,
            ),
        )

        NamedCommands.registerCommand(
            "FeedShooter5s",
            SequentialCommandGroup(
                WaitCommand(0.1),
                WaitCommand(3).until(
                    lambda: (
                        self.shooter.atFlywheelSetpoint()
                        and self.shooter.atHoodSetpoint()
                        and self.turret.atSetpoint()
                    )
                    or RobotBase.isSimulation()
                ),
                RepeatCommand(
                    ParallelCommandGroup(
                        IndexerShoot(self.indexer),
                        WoahvalScore(self.woahval),
                    )
                ).withTimeout(5),
            ),
        )

        NamedCommands.registerCommand(
            "StopFeedingShooter",
            ParallelCommandGroup(
                IndexerStop(self.indexer),
                WoahvalStop(self.woahval),
            ),
        )

        NamedCommands.registerCommand(
            "IntakeClearance", IntakeSetAngle(self.intake, 70)
        )
        NamedCommands.registerCommand("IntakeRetract", IntakeSetAngle(self.intake, 90))
        EventTrigger("IntakeDeploy").onTrue(
            SequentialCommandGroup(
                IntakeSetRollerSpeed(self.intake, 0.6),
                IntakeSetAngle(self.intake, 60),
                WaitCommand(0.375),
                IntakeSetAngle(self.intake, 0),
            )
        )

    def timerPeriodic(self) -> None:
        self.timerManager.periodic()

    def getAutoCommand(self) -> Command:
        return self.auto_chooser.getSelected()
