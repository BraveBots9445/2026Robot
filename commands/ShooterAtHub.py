from typing import Callable

from phoenix6 import swerve

from subsystems.shooter import Shooter

from tools.rebuilt import Rebuilt, RebuiltPositions

from .ShooterAdjustToTarget import ShooterAdjustToTarget


class ShooterAtHub(ShooterAdjustToTarget):
    # Interpolation tables map distance in meters -> (flywheel RPM, hood angle degrees).
    interpTable: dict[float, tuple[float, float]] = {
        1.5913605690002441: (2012.1563720703125, 72.971435546875),
        2.0009851455688477: (2108.6103515625, 72.974853515625),
        2.5068445205688477: (2157.917236328125, 72.97509765625),
        3.095264196395874: (2472.768310546875, 72.974853515625),
        4.179599761962891: (2717.844482421875, 70.86328125),
        4.624223709106445: (2873.511962890625, 70.884765625),
        5.331846714019775: (2990.961181640625, 70.950439453125),
        5.846864700317383: (3346.54541015625, 71.347900390625),
        6.928900718688965: (3697.115478515625, 71.453857421875),
        6.936418056488037: (3339.85009765625, 68.6064453125),
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
