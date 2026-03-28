from typing import Callable
import math

from commands2 import Command
from wpimath.controller import PIDController

from ntcore import NetworkTableInstance

from wpimath.geometry import Pose2d
from wpimath.units import meters_per_second, degrees_per_second, degreesToRadians

from phoenix6 import swerve

from subsystems.ctredrivetrain import CommandSwerveDrivetrain


class DriveToPose(Command):
    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        getX: Callable[[], float],
        getY: Callable[[], float],
        getRotation: Callable[[], float],
        getGoal: Callable[[], Pose2d],
        fieldCentric: bool = True,
    ):
        """
        Drive the robot toward a target pose using translational PID control.

        Target heading is commanded from the goal pose rotation.
        Deadbands should be handled by the controller input functions.

        :param drivetrain: Drivetrain subsystem to command.
        :type drivetrain: CommandSwerveDrivetrain
        :param getX: Returns forward/backward stick input.
        :type getX: Callable[[], float]
        :param getY: Returns left/right stick input.
        :type getY: Callable[[], float]
        :param getRotation: Returns rotational stick input.
        :type getRotation: Callable[[], float]
        :param getGoal: Returns the target field-relative pose.
        :type getGoal: Callable[[], Pose2d]
        :param fieldCentric: True for field-relative velocity commands.
        :type fieldCentric: bool
        """
        self.drivetrain = drivetrain
        self._field_centric = fieldCentric

        self.getX = getX
        self.getY = getY
        self.getRotation = getRotation
        self.getGoal = getGoal

        # Tunable defaults for translation control.
        self._x_pid = PIDController(2.0, 0.0, 0.0)
        self._y_pid = PIDController(2.0, 0.0, 0.0)

        self._xy_tolerance_m = 0.05
        self._theta_tolerance_deg = 3.0
        self._override_deadband = 0.08

        self._active_goal: Pose2d | None = None

        if fieldCentric:
            self._drive = swerve.requests.FieldCentricFacingAngle().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            )
        else:
            self._drive = swerve.requests.RobotCentricFacingAngle().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            )

        self.setName(f"DriveToPose-{ 'FieldOriented' if fieldCentric else 'RobotRelative' }")
        self.addRequirements(self.drivetrain)

    def initialize(self):
        self._x_pid.reset()
        self._y_pid.reset()
        self._active_goal = None

    def execute(self):
        maxLinearSpeed = self.drivetrain.getMaxSpeed()

        pose = self.drivetrain.get_state().pose
        goal = self.getGoal()
        self._active_goal = goal

        # Field-frame translational PID commands.
        vx_field = self._x_pid.calculate(pose.X(), goal.X())
        vy_field = self._y_pid.calculate(pose.Y(), goal.Y())

        vx_field = max(-maxLinearSpeed, min(maxLinearSpeed, vx_field))
        vy_field = max(-maxLinearSpeed, min(maxLinearSpeed, vy_field))

        if self._field_centric:
            vx_cmd = vx_field
            vy_cmd = vy_field
        else:
            # Convert field-relative velocities into robot-relative velocities.
            theta = pose.rotation().radians()
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            vx_cmd = vx_field * cos_t + vy_field * sin_t
            vy_cmd = -vx_field * sin_t + vy_field * cos_t

        self.drivetrain.set_control(
            self._drive.with_velocity_x(vx_cmd)
            .with_velocity_y(vy_cmd)
            .with_target_direction(goal.rotation())
        )

    def isFinished(self) -> bool:
        # Driver input overrides this command immediately.
        if (
            abs(self.getRotation()) > self._override_deadband
            or abs(self.getX()) > self._override_deadband
            or abs(self.getY()) > self._override_deadband
        ):
            return True

        if self._active_goal is None:
            return False

        pose = self.drivetrain.get_state().pose
        x_err = abs(self._active_goal.X() - pose.X())
        y_err = abs(self._active_goal.Y() - pose.Y())
        theta_err_deg = abs(
            math.remainder(
                self._active_goal.rotation().degrees() - pose.rotation().degrees(), 360.0
            )
        )

        return (
            x_err <= self._xy_tolerance_m
            and y_err <= self._xy_tolerance_m
            and theta_err_deg <= self._theta_tolerance_deg
        )
