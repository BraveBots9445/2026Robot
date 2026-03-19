from typing import Callable

from threading import Thread

from ntcore import (
    NetworkTable,
    NetworkTableInstance,
    StructArrayPublisher,
)

from wpilib import Notifier, RobotBase, Timer

from wpimath.geometry import Pose3d, Transform3d, Rotation2d, Translation3d, Rotation3d
from wpimath.units import seconds, degreesToRadians, inchesToMeters


class Visualizer3D:
    """
    A class to visualize 3D poses and transforms for the robot.
    It sends Pose3ds via networktables for visualiztation in AdvantageScope with 3d models of the robot included
    """

    _nettable: NetworkTable

    _mechPosePub: StructArrayPublisher
    """
    A publisher for the mechanism poses struct.
    Publishes in array[Pose3d] 
    """

    ########## POSES ##########
    # all poses are relative to the robot center (origin)

    _climberInitialPose = Pose3d(
        Translation3d(inchesToMeters(-10), inchesToMeters(0.125), inchesToMeters(6.25)),
        Rotation3d(),
    )
    """
    The initial pose of the climber elevator (when viewed from the front of the robot).
    """

    _climberCarriageInitialPose = Pose3d(
        Translation3d(inchesToMeters(-10), inchesToMeters(0.5), inchesToMeters(7.375)),
        Rotation3d(0, 0, degreesToRadians(90)),
    )

    _intakeInitialPose = Pose3d(
        Translation3d(inchesToMeters(11.5), 0, inchesToMeters(7.65)), Rotation3d()
    )
    """
    The initial position of the intake mechanism along its pivot axis (TODO: is it pivoting or linear?).
    """

    _hopperInitialPose = Pose3d()
    """
    The initial position of the extending hopper mechanism relative to the robot 
    """

    _turretInitialPose = Pose3d(
        Translation3d(
            inchesToMeters(0),
            inchesToMeters(6.0),
            inchesToMeters(12.914),
        ),
        Rotation3d(),
    )
    """
    The initial pose of the turret mechanism.
    """

    _hoodInitialPose = Pose3d(
        Translation3d(
            inchesToMeters(-0.75),
            inchesToMeters(4.5),
            inchesToMeters(13.75),
        ),
        Rotation3d(),
    )

    """
    The initial pose of the hood mechanism.
    This should be at the minimum angle position.
    """

    _hoodRotationTranslation: Translation3d = Translation3d(
        inchesToMeters(4.3), 0, inchesToMeters(1.436)
    )
    """
    The translation from the hood's initial pose to its axis of rotation 
    """

    ########## GETTERS ##########
    _getClimberTransform: Callable[[], Transform3d]
    """
    Returns the current transform of the climber elevator from its initial pose.
    """

    _getIntakeTransform: Callable[[], Transform3d]
    """
    Returns the current transform of the intake mechanism from its initial pose.
    """

    _getExtendingHopperTransform: Callable[[], Transform3d]
    """
    Returns the current transform of the extending hopper mechanism from its initial pose.
    """

    _getTurretTransform: Callable[[], Transform3d]
    """
    Returns the current transform of the turret mechanism from its initial pose.
    """

    _getHoodAngle: Callable[[], Rotation2d]
    """
    Returns the current angle of the hood mechanism.
    """

    def __init__(
        self,
        getClimberTransform: Callable[[], Transform3d],
        getIntakeTransform: Callable[[], Transform3d],
        getExtendingHopperTransform: Callable[[], Transform3d],
        getTurretTransform: Callable[[], Transform3d],
        getHoodAngle: Callable[[], Rotation2d],
        period: seconds = 0.02,
    ):
        """
        Initializes the Visualizer3D with functions to get the current transforms

        :param getClimberTransform: Function to get the climber transform
        :type getClimberTransform: Callable[[], Transform3d]
        :param getClimberRightTransform: Function to get the right climber transform
        :type getClimberRightTransform: Callable[[], Transform3d]
        :param getIntakeTransform: Function to get the intake transform
        :type getIntakeTransform: Callable[[], Transform3d]
        :param getTurretTransform: Function to get the turret transform
        :type getTurretTransform: Callable[[], Transform3d]
        :param getHoodAngle: Function to get the hood angle
        :type getHoodAngle: Callable[[], Rotation2d]
        :param period: The period at which to update the visualization, defaults to 0.02s
        :type period: seconds, optional
        """
        self._getClimberTransform = getClimberTransform
        self._getExtendingHopperTransform = getExtendingHopperTransform
        self._getIntakeTransform = getIntakeTransform
        self._getTurretTransform = getTurretTransform
        self._getHoodAngle = getHoodAngle

        self._nettable = NetworkTableInstance.getDefault().getTable(
            "000MechVisualizer3D"
        )
        self._mechPosePub = self._nettable.getStructArrayTopic(
            "MechPoses", Pose3d
        ).publish()
        self._publish_period = 0.05 if RobotBase.isSimulation() else 0.10
        self._last_publish_time = float("-inf")

        Thread(target=self.update, daemon=True, name="Visualizer3D")

    def update(self) -> None:
        """
        Update the mechanism poses and publish them to the network table.
        Call this from the main robot thread (e.g., robotPeriodic or a Subsystem's periodic).

        :return: None
        """
        now = Timer.getFPGATimestamp()
        if now - self._last_publish_time < self._publish_period:
            return
        self._last_publish_time = now

        turretTransform = self._getTurretTransform()
        hoodTransformation = self._hoodInitialPose.relativeTo(
            self._turretInitialPose
        ).rotateBy(
            # self._hoodRotationTranslation,
            Rotation3d(
                0, self._getHoodAngle().radians(), turretTransform.rotation().Z()
            ),
        )
        mechPoses = [
            Pose3d(),
            self._intakeInitialPose.transformBy(self._getIntakeTransform()),
            self._climberInitialPose.transformBy(self._getClimberTransform() / 2),
            self._climberCarriageInitialPose.transformBy(self._getClimberTransform()),
            self._turretInitialPose.transformBy(turretTransform),
            Pose3d(
                self._hoodInitialPose.translation() + hoodTransformation.translation(),
                hoodTransformation.rotation(),
            ),
        ]

        self._mechPosePub.set(mechPoses)

    @property
    def transform3dToTurret(self) -> Transform3d:
        """
        A property to get the current transform from the robot center to the turret.

        :return: The current transform from the robot center to the turret.
        :rtype: Transform3d
        """
        return Transform3d(self._turretInitialPose.translation(), Rotation3d())
