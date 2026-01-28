from commands2 import (
    Command,
    InstantCommand,
    cmd,
    Subsystem
)
from phoenix6 import swerve

from wpimath import applyDeadband
from wpimath.geometry import Transform2d, Rotation2d, Translation2d, Pose3d, Translation3d, Rotation3d
from wpimath.units import inchesToMeters

# from subsystems.vision import Vision
from telemetry import Telemetry
from generated.tuner_constants import TunerConstants

from commands2.button import CommandXboxController

from ntcore import NetworkTableInstance
from ntcore.util import ntproperty

from wpilib import PowerDistribution, SmartDashboard, DriverStation, RobotBase

from pathplannerlib.auto import AutoBuilder, NamedCommands, PathConstraints

from subsystems.turret import Turret
from commands.turretByStick import TurretByStick
from commands.turretToRotation import TurretToRotation
from commands.turretAtTarget import TurretAtTarget
from commands.turretFollowTarget import TurretFollowTarget
from commands.launchCalculator import LaunchCalculator
from commands.prepareShooterEstimate import PrepareShooterEstimate

from tools.FuelVisualizer import Fuel, FuelVisualizer

class RobotContainer:
    _max_speed_percent = ntproperty("MaxVelocityPercent", 1.0)
    _max_angular_rate_percent = ntproperty("MaxOmegaPercent", 1.0)

    _max_speed = TunerConstants.speed_at_12_volts
    _max_angular_rate = 0.75  # radians per second

    def __init__(self) -> None:
        DriverStation.silenceJoystickConnectionWarning(True)
        self.driver_controller = CommandXboxController(0)
        self.operator_controller = CommandXboxController(1)
        self.pdh = PowerDistribution()
        self.pdh.setSwitchableChannel(True)
        self.nettable = NetworkTableInstance.getDefault().getTable("0000DriverInfo")

        self.level = 1

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(0)  # deadband is handled in get_velocity_x/y
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )  # Use open-loop control for drive motors

        self._robot_drive = (
            swerve.requests.RobotCentric()
            .with_deadband(0)  # deadband is handled in get_velocity_x/y
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )  # Use open-loop control for drive motors

        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self.drivetrain = TunerConstants.create_drivetrain()

        # self.vision = Vision(
        #     self.drivetrain.add_vision_measurement,
        #     lambda: self.drivetrain.get_state().pose,
        #     lambda: self.drivetrain.get_state().speeds,
        # )

        self.drivetrain.register_telemetry(
            lambda telem: self._logger.telemeterize(telem)
        )

        self.set_pp_named_commands()

        self.auto_chooser = AutoBuilder.buildAutoChooser()

        self.turret = Turret()
        self.turret.setRobotPoseGetter( lambda: self.drivetrain.get_state().pose )

        # if RobotBase.isSimulation():
        self.fuelVisual = FuelVisualizer()

        SmartDashboard.putData(self.auto_chooser)
        SmartDashboard.putData(self.drivetrain)
        SmartDashboard.putData(self.turret)

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

        # self.driver_controller.x().onTrue(
        #     self.vision.toggle_vision_measurements_command()
        # )
       
        ### TURRET TESTING
        self.turret.setDefaultCommand(
            TurretByStick( self.turret, lambda: self.driver_controller.getLeftTriggerAxis() - self.driver_controller.getRightTriggerAxis() )
        )
        self.driver_controller.y().onTrue( TurretToRotation( self.turret, 0 ) )
        self.driver_controller.x().onTrue( TurretToRotation( self.turret, 90 ) )
        self.driver_controller.a().onTrue( TurretToRotation( self.turret, 180 ) )
        self.driver_controller.b().onTrue( TurretToRotation( self.turret, -90 ) )
        self.driver_controller.rightBumper().toggleOnTrue( TurretAtTarget( self.turret, self.drivetrain.get_state, Translation2d( 4.75, 8.17/2 ) ) )
        self.driver_controller.leftBumper().toggleOnTrue( TurretFollowTarget( self.turret, self.drivetrain.get_state, Translation2d( 4.75, 8.17/2 ) ) )
        
        ### FUEL SIMULATION
        self.driver_controller.start().onTrue( cmd.runOnce( lambda: self.new_fuel() ) )
        
        ### LAUNCH TESTING
        self.driver_controller.povUp().whileTrue( LaunchCalculator( self.turret.getAbsolutePose ) )
        hoodSys = Subsystem()
        SmartDashboard.putData( "Hood", hoodSys )
        shooterSys = Subsystem()
        SmartDashboard.putData( "Shooter", shooterSys )
        self.driver_controller.povDown().whileTrue( PrepareShooterEstimate( hoodSys, shooterSys, self.turret.getAbsolutePose ) )
        """Operator"""
        """
        Insert code here for the secondary driver
        """

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

    def new_fuel(self) -> None:
        #self.fuelVisual.newFuel( Pose3d(8,4,4,Rotation3d()), Translation3d(0,0,0.0) )
        self.fuelVisual.randomFuel()