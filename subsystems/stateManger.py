from dataclasses import dataclass

from enum import Enum

from commands2 import Subsystem, Command, SelectCommand, cmd
from commands2.button import Trigger

from ntcore import NetworkTableInstance, NetworkTable, StringPublisher

from wpimath.units import inchesToMeters
from wpimath.geometry import Translation2d, Rectangle2d, Pose2d, Rotation2d, Pose3d

from wpilib import Notifier

from wpiutil.wpistruct import make_wpistruct

from commands.stateTransitionCommands.aimToShoot import AimToShoot
from commands.stateTransitionCommands.shootToAim import ShootToAim
from commands.stateTransitionCommands.climbLowToIntake import ClimbLowToIntake
from commands.stateTransitionCommands.intakeToClimbLow import IntakeToClimbLow
from commands.stateTransitionCommands.intakeToClimbHigh import IntakeToClimbHigh
from commands.stateTransitionCommands.intakeToNone import IntakeToNone
from commands.stateTransitionCommands.toStow import ToStow
from commands.stateTransitionCommands.noneToClimbLow import NoneToClimbLow
from commands.stateTransitionCommands.noneToClimbHigh import NoneToClimbHigh
from commands.stateTransitionCommands.noneToIntake import NoneToIntake
from commands.stateTransitionCommands.climbLowToClimbHigh import ClimbLowToClimbHigh
from commands.stateTransitionCommands.noneToShoot import NoneToShoot
from commands.stateTransitionCommands.noneToAim import NoneToAim
from commands.stateTransitionCommands.aimToNone import AimToNone

from subsystems import (
    CommandSwerveDrivetrain,
    Shooter,
    Turret,
    Kicker,
    Indexer,
    Woahval,
    Climber,
    Intake,
    PassiveHooks,
    ShootOnMoveCalculator,
)

from tools.rebuilt import Rebuilt


class ShootingState(Enum):
    NONE = 0
    AIMING = 1
    SHOOTING = 2
    STOWED = 3


class ExtendingState(Enum):
    NONE = 0  # intake and climber stowed
    INTAKING = 1
    CLIMBING_LOW = 2
    CLIMBING_HIGH = 3


@make_wpistruct
@dataclass
class TriggerStates:
    mustStow: bool
    inAllianceZone: bool


class StateManager(Subsystem):
    """
    The state manager is responsible for keeping track of the current state of the robot
    and transitioning between states.
    """

    _drivetrain: CommandSwerveDrivetrain
    _shooter: Shooter
    _turret: Turret
    _kicker: Kicker
    _indexer: Indexer
    _woahval: Woahval
    _climber: Climber
    _intake: Intake
    _passiveHooks: PassiveHooks
    _shootOnMoveCalculator: ShootOnMoveCalculator

    _nettable: NetworkTable

    _extendingStatePub: StringPublisher
    _shootingStatePub: StringPublisher

    _pubNotifier: Notifier

    _extendingState: ExtendingState = ExtendingState.NONE
    _shootingState: ShootingState = ShootingState.NONE

    _trenchZones = [
        Rectangle2d(
            Translation2d(inchesToMeters(100), inchesToMeters(0)),
            Translation2d(inchesToMeters(260), inchesToMeters(65)),
        ),
        Rectangle2d(
            Translation2d(inchesToMeters(100), Rebuilt.Width - inchesToMeters(0)),
            Translation2d(inchesToMeters(260), Rebuilt.Width - inchesToMeters(65)),
        ),
        Rectangle2d(
            Translation2d(Rebuilt.Length - inchesToMeters(100), inchesToMeters(0)),
            Translation2d(Rebuilt.Length - inchesToMeters(260), inchesToMeters(65)),
        ),
        Rectangle2d(
            Translation2d(
                Rebuilt.Length - inchesToMeters(100),
                Rebuilt.Width - inchesToMeters(0),
            ),
            Translation2d(
                Rebuilt.Length - inchesToMeters(260),
                Rebuilt.Width - inchesToMeters(65),
            ),
        ),
    ]

    _allianceZone = Rectangle2d(
        Rebuilt.getPosition(
            Pose3d(
                Pose2d(inchesToMeters(91.055), inchesToMeters(158.845), Rotation2d(0))
            )
        ).toPose2d(),
        inchesToMeters(182.11),
        inchesToMeters(317.69),
    )

    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        shooter: Shooter,
        turret: Turret,
        kicker: Kicker,
        indexer: Indexer,
        woahval: Woahval,
        climber: Climber,
        intake: Intake,
        passiveHooks: PassiveHooks,
        shootOnMoveCalculator: ShootOnMoveCalculator,
    ):
        self._drivetrain = drivetrain
        self._shooter = shooter
        self._turret = turret
        self._kicker = kicker
        self._indexer = indexer
        self._woahval = woahval
        self._climber = climber
        self._intake = intake
        self._passiveHooks = passiveHooks
        self._shootOnMoveCalculator = shootOnMoveCalculator

        self._nettable = NetworkTableInstance.getDefault().getTable("000State")
        self._extendingStatePub = self._nettable.getStringTopic(
            "extendingState"
        ).publish()
        self._shootingStatePub = self._nettable.getStringTopic(
            "shootingState"
        ).publish()
        self._zonesCenterPub = self._nettable.getStructArrayTopic(
            "zonesCenters", Pose2d
        ).publish()
        self._triggerStatesPub = self._nettable.getStructTopic(
            "triggerStates", TriggerStates
        ).publish()

        self._pubNotifier = Notifier(self.publish)
        self._pubNotifier.startPeriodic(0.5)  # state names change rarely, 500ms is fine

        self._zonesCenterPub.set([rect.center() for rect in self._trenchZones])

    def publish(self) -> None:
        self._extendingStatePub.set(self._extendingState.name)
        self._shootingStatePub.set(self._shootingState.name)
        self._triggerStatesPub.set(
            TriggerStates(self.getMustStowBool(), self.getInAllianceZoneBool())
        )

    def startIntaking(self) -> Command:
        def update():
            # Only commenting this here - this is a hack to update the state whenever the command is accessed
            startState = self._extendingState
            if self._extendingState != ExtendingState.CLIMBING_HIGH:
                self._extendingState = ExtendingState.INTAKING
            return startState

        return SelectCommand(
            {
                ExtendingState.NONE: NoneToIntake(self._intake),
                ExtendingState.INTAKING: cmd.none(),  # already intaking, do nothing
                ExtendingState.CLIMBING_LOW: ClimbLowToIntake(
                    self._climber, self._intake
                ),
                ExtendingState.CLIMBING_HIGH: cmd.none(),  # ClimbHigh is not possible because we can't climb down from high, and extension isn't allowed by intake when climbed
                # we use none to prevent a crash
            },
            lambda: update(),
        )

    def stopIntaking(self) -> Command:
        def update():
            startState = self._extendingState
            if self._extendingState == ExtendingState.INTAKING:
                self._extendingState = ExtendingState.NONE
                # not all states becuase we don't want to override climbing
            return startState

        return SelectCommand(
            {
                ExtendingState.NONE: cmd.none(),  # not intaking, do nothing
                ExtendingState.INTAKING: IntakeToNone(self._intake),
                ExtendingState.CLIMBING_LOW: cmd.none(),
                ExtendingState.CLIMBING_HIGH: cmd.none(),
            },
            lambda: update(),
        )

    def startClimbingLow(self) -> Command:
        def update():
            startState = self._extendingState
            self._extendingState = ExtendingState.CLIMBING_LOW
            return startState

        return SelectCommand(
            {
                ExtendingState.NONE: NoneToClimbLow(self._climber),
                ExtendingState.INTAKING: IntakeToClimbLow(self._climber, self._intake),
                ExtendingState.CLIMBING_LOW: cmd.none(),  # already climbing low, do nothing
                ExtendingState.CLIMBING_HIGH: ClimbLowToClimbHigh(
                    self._climber, self._passiveHooks
                ),
            },
            lambda: update(),
        )

    def startClimbingHigh(self) -> Command:
        def update():
            startState = self._extendingState
            self._extendingState = ExtendingState.CLIMBING_HIGH
            return startState

        return SelectCommand(
            {
                ExtendingState.NONE: NoneToClimbHigh(self._climber, self._passiveHooks),
                ExtendingState.INTAKING: IntakeToClimbHigh(
                    self._climber, self._intake, self._passiveHooks
                ),
                ExtendingState.CLIMBING_LOW: ClimbLowToClimbHigh(
                    self._climber, self._passiveHooks
                ),
                ExtendingState.CLIMBING_HIGH: cmd.none(),  # already climbing high, do nothing
            },
            lambda: update(),
        )

    def stow(self) -> Command:
        def update():
            self._extendingState = ExtendingState.NONE
            self._shootingState = ShootingState.STOWED
            return 0

        return SelectCommand(
            {0: ToStow(self._intake, self._climber, self._shooter)},
            lambda: update(),
        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)

    def startAiming(self) -> Command:
        def update():
            startState = self._shootingState
            self._shootingState = ShootingState.AIMING
            return startState

        return SelectCommand(
            {
                ShootingState.NONE: NoneToAim(
                    self._shooter,
                    self._turret,
                    self._kicker,
                    self._indexer,
                    self._woahval,
                    self._shootOnMoveCalculator,
                ),
                ShootingState.AIMING: cmd.none(),  # already aiming, do nothing
                ShootingState.SHOOTING: ShootToAim(
                    self._kicker,
                    self._indexer,
                    self._woahval,
                    self._shooter,
                    self._turret,
                    self._shootOnMoveCalculator,
                ),
                ShootingState.STOWED: NoneToAim(
                    self._shooter,
                    self._turret,
                    self._kicker,
                    self._indexer,
                    self._woahval,
                    self._shootOnMoveCalculator,
                ),
            },
            lambda: update(),
        )

    def startShooting(self) -> Command:
        def update():
            startState = self._shootingState
            self._shootingState = ShootingState.SHOOTING
            return startState

        return SelectCommand(
            {
                ShootingState.NONE: NoneToShoot(
                    self._shooter,
                    self._turret,
                    self._kicker,
                    self._indexer,
                    self._woahval,
                    self._shootOnMoveCalculator,
                ),
                ShootingState.AIMING: AimToShoot(
                    self._kicker,
                    self._indexer,
                    self._woahval,
                    self._shooter,
                    self._turret,
                    self._shootOnMoveCalculator,
                ),
                ShootingState.SHOOTING: cmd.none(),  # already shooting, do nothing
                ShootingState.STOWED: NoneToShoot(
                    self._shooter,
                    self._turret,
                    self._kicker,
                    self._indexer,
                    self._woahval,
                    self._shootOnMoveCalculator,
                ),
            },
            lambda: update(),
        )

    def stopShooting(self) -> Command:
        def update():
            startState = self._shootingState
            self._shootingState = ShootingState.NONE
            return startState

        return SelectCommand(
            {
                ShootingState.NONE: cmd.none(),  # not shooting, do nothing
                ShootingState.AIMING: AimToNone(self._woahval, self._indexer),
                ShootingState.SHOOTING: AimToNone(
                    self._woahval, self._indexer
                ),  # we can reuse AimToNone because it just stops the indexer and woahval, which is what we want when stopping shooting from either state
            },
            lambda: update(),
        )

    def getMustStowTrigger(self) -> Trigger:
        return Trigger(self.getMustStowBool)

    def getMustStowBool(self) -> bool:
        pose = self._drivetrain.get_state().pose
        for zone in self._trenchZones:
            if zone.contains(pose.translation()):
                return True
        return False

    def getInAllianceZoneBool(self) -> bool:
        pose = self._drivetrain.get_state().pose
        return self._allianceZone.contains(pose.translation())

    def getInAllianceZoneTrigger(self) -> Trigger:
        return Trigger(self.getInAllianceZoneBool)

    def resetAllianceZone(self) -> None:
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

    def resetAllianceZoneCommand(self) -> Command:
        return cmd.runOnce(self.resetAllianceZone, self).ignoringDisable(True)
