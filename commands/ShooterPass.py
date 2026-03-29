from typing import Callable
from enum import Enum

from phoenix6 import swerve
from wpimath.geometry import Pose2d

from subsystems.shooter import Shooter
from tools.rebuilt import Rebuilt, RebuiltPositions

from .ShooterAdjustToTarget import ShooterAdjustToTarget


class ShooterPass(ShooterAdjustToTarget):
    class Target:
        LEFT = Rebuilt.getPosition(RebuiltPositions.PassLeft).toPose2d()
        RIGHT = Rebuilt.getPosition(RebuiltPositions.PassRight).toPose2d()

    # Interpolation table maps distance in meters -> (flywheel RPM, hood angle degrees).
    interpTable: dict[float, tuple[float, float]] = {
        2.0: (2200.0, 14.0),
        3.0: (2400.0, 20.0),
        4.0: (2600.0, 26.0),
    }

    def __init__(
        self,
        shooter: Shooter,
        robotState: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState],
        targetPose: Pose2d,
        enableShootOnMove: bool = False,
    ):
        """
        Adjust shooter setpoints toward the passing target using ShooterAdjustToTarget.

        :param shooter: Shooter subsystem.
        :type shooter: Shooter
        :param robotState: Supplier of current Phoenix swerve drivetrain state.
        :type robotState: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState]
        :param targetPose: Target pass position pose.
        :type targetPose: Pose2d
        :param enableShootOnMove: Placeholder option for future shoot-on-the-move support.
        :type enableShootOnMove: bool
        """
        super().__init__(
            shooter,
            robotState,
            targetPose,
            self.interpTable,
            enableShootOnMove,
        )
        self.setName("ShooterPass")
