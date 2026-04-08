from typing import Callable

from phoenix6 import swerve

from subsystems.shooter import Shooter

from tools.rebuilt import Rebuilt, RebuiltPositions

from .ShooterAdjustToTarget import ShooterAdjustToTarget


class ShooterAtHub(ShooterAdjustToTarget):
    # Interpolation tables map distance in meters -> (flywheel RPM, hood angle degrees).
    _flywheelFudge: float = 1.01
    interpTable: dict[float, tuple[float, float]] = {
        2.0115275382995605: (2211.5009765625, 72.977783203125),
        2.489119291305542: (2253.2548828125, 72.9775390625),
        2.870438575744629: (2380.037109375, 72.977783203125),
        3.581763505935669: (2500.574951171875, 72.977783203125),
        4.130230903625488: (2607.271484375, 73.016845703125),
        4.617537975311279: (2816.05029296875, 73.016845703125),
        5.423771381378174: (3002.201904296875, 72.489990234375),
        5.7846879959106445: (3084.089111328125, 72.489990234375),
        6.9759840965271: (3510.734130859375, 72.551513671875),
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
