from typing import Callable

from commands2 import Command
import numpy as np
from phoenix6 import swerve

from wpimath.geometry import Pose2d

from subsystems.shooter import Shooter


class ShooterAdjustToTarget(Command):
    def __init__(
        self,
        shooter: Shooter,
        robotState: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState],
        targetPose: Pose2d,
        interpolationTable: dict[float, tuple[float, float]],
        enableShootOnMove: bool = False,
    ):
        """
        Continuously adjust shooter setpoints based on distance to target.

        :param shooter: Shooter subsystem.
        :type shooter: Shooter
        :param robotState: Supplier of current Phoenix swerve drivetrain state.
            This should be `Callable[[], swerve.SwerveDrivetrain.SwerveDriveState]`,
            such as `lambda: drivetrain.get_state()`.
        :type robotState: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState]
        :param targetPose: Target pose to shoot at.
        :type targetPose: Pose2d
        :param interpolationTable: Distance-to-setpoint lookup table where keys are
            distance meters and values are (flywheelRpm, hoodDeg). This table is
            snapshotted once at command creation and is not updated afterward.
        :type interpolationTable: dict[float, tuple[float, float]]
        :param enableShootOnMove: Placeholder option for future shoot-on-the-move support.
        :type enableShootOnMove: bool
        """
        self.shooter = shooter
        self.robotState = robotState
        self.targetPose = targetPose
        self._interpDistances = np.array([])
        self._interpFlywheelRpms = np.array([])
        self._interpHoodDegrees = np.array([])

        # Snapshot table values now so interpolation remains static for this command instance.
        sortedRows = sorted(interpolationTable.items())
        if sortedRows:
            self._interpDistances = np.array([row[0] for row in sortedRows], dtype=float)
            self._interpFlywheelRpms = np.array([row[1][0] for row in sortedRows], dtype=float)
            self._interpHoodDegrees = np.array([row[1][1] for row in sortedRows], dtype=float)

        # Stored for future shoot-on-the-move compensation logic.
        self.enableShootOnMove = enableShootOnMove

        self.setName("ShooterAdjustToTarget")
        self.addRequirements(self.shooter)

    def _lookupSetpoints(self, distanceMeters: float) -> tuple[float, float]:
        if self._interpDistances.size == 0:
            return 0.0, self.shooter.minHoodAngle.degrees()

        rpm = float(np.interp(distanceMeters, self._interpDistances, self._interpFlywheelRpms))
        deg = float(np.interp(distanceMeters, self._interpDistances, self._interpHoodDegrees))
        return rpm, deg

    def initialize(self):
        self.execute()

    def _getRobotPoseFromState(self) -> Pose2d:
        state = self.robotState()
        return state.pose

    def execute(self):
        robotPose = self._getRobotPoseFromState()

        distanceMeters = robotPose.translation().distance(
            self.targetPose.translation()
        )
        flywheelRpm, hoodDegrees = self._lookupSetpoints(distanceMeters)

        self.shooter.setFlywheelSetpoint(flywheelRpm)
        self.shooter.setHoodAngleSetpointDegrees(hoodDegrees)

    def isFinished(self) -> bool:
        return False
