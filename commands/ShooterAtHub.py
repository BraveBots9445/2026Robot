from typing import Callable

from phoenix6 import swerve

from subsystems.shooter import Shooter

from tools.rebuilt import Rebuilt, RebuiltPositions

from .ShooterAdjustToTarget import ShooterAdjustToTarget


class ShooterAtHub(ShooterAdjustToTarget):
    # Interpolation tables map distance in meters -> (flywheel RPM, hood angle degrees).
    interpTable: dict[float, tuple[float, float]] = {
        1.5: (3000.0, 32.0),
        2.5: (3400.0, 42.0),
        3.5: (3800.0, 52.0),
    }

    def __init__(
        self,
        shooter: Shooter,
        robotState: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState],
        enableShootOnMove: bool = False,
    ):
        """
        Adjust shooter setpoints toward the Hub target using ShooterAdjustToTarget.

        :param shooter: Shooter subsystem.
        :type shooter: Shooter
        :param robotState: Supplier of current Phoenix swerve drivetrain state.
        :type robotState: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState]
        :param interpolationTable: Static distance-to-setpoint table.
        :type interpolationTable: dict[float, tuple[float, float]]
        :param enableShootOnMove: Placeholder option for future shoot-on-the-move support.
        :type enableShootOnMove: bool
        """
        super().__init__(
            shooter,
            robotState,
            Rebuilt.getPosition(RebuiltPositions.Hub).toPose2d(),
            self.interpTable,
            enableShootOnMove,
        )
        self.setName("ShooterAtHub")
