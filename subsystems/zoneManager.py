from dataclasses import dataclass

from commands2 import Subsystem, Command, cmd
from commands2.button import Trigger

from ntcore import NetworkTableInstance, NetworkTable, StructPublisher

from wpimath.units import inchesToMeters, meters, seconds
from wpimath.geometry import Translation2d, Rectangle2d, Pose2d, Rotation2d, Pose3d

from wpiutil.wpistruct import make_wpistruct

from subsystems import (
    CommandSwerveDrivetrain,
)

from tools.rebuilt import Rebuilt


@make_wpistruct
@dataclass
class ZoneStates:
    mustStowPredict: bool
    inAllianceZone: bool
    onLeft: bool
    onRight: bool


class ZoneManager(Subsystem):
    """
    The state manager is responsible for keeping track of the current state of the robot
    and transitioning between states.
    """

    _drivetrain: CommandSwerveDrivetrain

    _nettable: NetworkTable

    _trenchLength: meters = inchesToMeters(67.0)
    _trenchWidth: meters = inchesToMeters(59.84)

    _trenchZones = [
        Rectangle2d(
            Pose2d(inchesToMeters(182.11), inchesToMeters(49.84 / 2), Rotation2d()),
            _trenchLength,
            _trenchWidth,
        ),
        Rectangle2d(
            Pose2d(
                inchesToMeters(182.11),
                Rebuilt.Width - inchesToMeters(49.84 / 2),
                Rotation2d(),
            ),
            _trenchLength,
            _trenchWidth,
        ),
        Rectangle2d(
            Pose2d(
                Rebuilt.Length - inchesToMeters(182.11),
                inchesToMeters(49.84 / 2),
                Rotation2d(),
            ),
            _trenchLength,
            _trenchWidth,
        ),
        Rectangle2d(
            Pose2d(
                Rebuilt.Length - inchesToMeters(182.11),
                Rebuilt.Width - inchesToMeters(49.84 / 2),
                Rotation2d(),
            ),
            _trenchLength,
            _trenchWidth,
        ),
    ]

    _allianceZone: Rectangle2d

    _leftZone: Rectangle2d

    _statePub: StructPublisher

    _state: ZoneStates

    def __init__(self, drivetrain: CommandSwerveDrivetrain):
        self._drivetrain = drivetrain

        self._nettable = NetworkTableInstance.getDefault().getTable("000Zones")

        self._state = ZoneStates(False, False, False, False)

        self._zonesCenterPub = self._nettable.getStructArrayTopic(
            "zonesCenters", Pose2d
        ).publish()

        self._statePub = self._nettable.getStructTopic("state", ZoneStates).publish()

        self.resetZonesByAlliance()

    def periodic(self) -> None:
        self._state.mustStowPredict = self._willMustStow()
        self._state.inAllianceZone = self._getInAllianceZone()
        self._state.onLeft = self._leftZone.contains(
            self._drivetrain.get_state().pose.translation()
        )
        self._state.onRight = not self._state.onLeft
        self._statePub.set(self._state)

    def getMustStowTrigger(self, time: seconds = 0.75) -> Trigger:
        return Trigger(lambda: self.getMustStowBool(time))

    def _willMustStow(self, time: seconds = 0.75) -> bool:
        # TODO: If we are on the bump and drive towards the trench, this will trigger must stow even though we are not in the trench.
        # This should be fine, so we will just test and see if behavior is bad.
        # solution is to ignore if we are moving to the outside of the field within a certain x range
        state = self._drivetrain.get_state()
        pose = state.pose
        vel = state.speeds

        px = pose.X()
        py = pose.Y()

        vx = vel.vx
        vy = vel.vy

        if self._getMustStowBool(pose):
            return True

        if abs(vx) <= 0.1 and abs(vy) <= 0.1:
            return False

        for rect in self._trenchZones:
            center = rect.center()

            cx = center.X()
            cy = center.Y()

            xWidth = rect.xwidth / 2
            yWidth = rect.ywidth / 2

            xmin = cx - xWidth
            xmax = cx + xWidth
            ymin = cy - yWidth
            ymax = cy + yWidth

            # X slab
            if vx != 0:
                inv = 1 / vx
                tx1 = (xmin - px) * inv
                tx2 = (xmax - px) * inv
                txEnter = min(tx1, tx2)
                txExit = max(tx1, tx2)
            else:
                if not (xmin <= px <= xmax):
                    continue
                txEnter = float("-inf")
                txExit = float("inf")

            # Y slab
            if vy != 0:
                inv = 1 / vy
                ty1 = (ymin - py) * inv
                ty2 = (ymax - py) * inv
                tyEnter = min(ty1, ty2)
                tyExit = max(ty1, ty2)
            else:
                if not (ymin <= py <= ymax):
                    continue
                tyEnter = float("-inf")
                tyExit = float("inf")

            tEnter = max(txEnter, tyEnter)
            tExit = min(txExit, tyExit)

            if tEnter <= tExit and 0 <= tEnter <= time:
                return True

        return False

    def getMustStowBool(self, time: seconds = 0.75) -> bool:
        return self._willMustStow(time)

    def _getMustStowBool(self, pose: Pose2d) -> bool:
        for zone in self._trenchZones:
            if zone.contains(pose.translation()):
                return True
        return False

    def _getInAllianceZone(self) -> bool:
        pose = self._drivetrain.get_state().pose
        return self._allianceZone.contains(pose.translation())

    def getInAllianceZoneBool(self) -> bool:
        return self._state.inAllianceZone

    def getInAllianceZoneTrigger(self) -> Trigger:
        return Trigger(self.getInAllianceZoneBool)

    def getOnLeftBool(self) -> bool:
        return self._state.onLeft

    def getOnLeftTrigger(self) -> Trigger:
        return Trigger(self.getOnLeftBool)

    def getOnRightBool(self) -> bool:
        return self._state.onRight

    def getOnRightTrigger(self) -> Trigger:
        return Trigger(self.getOnRightBool)

    def resetZonesByAlliance(self) -> None:
        self._allianceZone = Rectangle2d(
            Rebuilt.getPosition(
                Pose3d(
                    Pose2d(
                        inchesToMeters(91.055), inchesToMeters(158.845), Rotation2d(0)
                    )
                )
            ).toPose2d(),
            inchesToMeters(182.11),
            inchesToMeters(317.69),
        )

        self._leftZone = Rectangle2d(
            Rebuilt.getPosition(
                Pose3d(
                    Pose2d(
                        inchesToMeters(651.22 / 2),
                        inchesToMeters(3 * 317.69 / 4),
                        Rotation2d(),
                    )
                )
            ).toPose2d(),
            inchesToMeters(1000),  # 651.22 is the actual length, but this is a buffer
            inchesToMeters(317.69 / 2),
        )

        self._zonesCenterPub.set(
            [
                rect.center()
                for rect in self._trenchZones + [self._leftZone, self._allianceZone]
            ]
        )

    def resetZonesCommand(self) -> Command:
        return cmd.runOnce(self.resetZonesByAlliance, self).ignoringDisable(True)
