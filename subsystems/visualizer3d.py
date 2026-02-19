from dataclasses import dataclass
from typing import Callable

from ntcore import (
    NetworkTable,
    NetworkTableInstance,
    StructArrayPublisher,
)

from wpilib import Notifier

from wpimath.geometry import Pose3d, Transform3d, Rotation2d, Translation3d, Rotation3d
from wpimath.units import seconds


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

    _notifier: Notifier
    """
    A notifier to periodically call the update method.
    """

    ########## POSES ##########
    # all poses are relative to the robot center (origin)

    _climberLeftInitialPose = Pose3d()
    """
    The initial pose of the left climber elevator (when viewed from behind the elevators).
    """

    _climberRightInitialPose = Pose3d()
    """
    The initial pose of the right climber elevator (when viewed from behind the elevators).
    """

    _intakeInitialPose = Pose3d()
    """
    The initial position of the intake mechanism along its pivot axis (TODO: is it pivoting or linear?).
    """

    _turretInitialPose = Pose3d()
    """
    The initial pose of the turret mechanism.
    """

    _hoodInitialPose = Pose3d()
    """
    The initial pose of the hood mechanism.
    This should be at the minimum angle position.
    """

    _hoodRotationTranslation = Translation3d()
    """
    The translation from the hood's initial pose to its axis of rotation 
    """

    ########## GETTERS ##########
    _getClimberLeftTransform: Callable[[], Transform3d]
    """
    Returns the current transform of the left climber elevator from its initial pose.
    """

    _getClimberRightTransform: Callable[[], Transform3d]
    """
    Returns the current transform of the right climber elevator from its initial pose.
    """

    _getIntakeTransform: Callable[[], Transform3d]
    """
    Returns the current transform of the intake mechanism from its initial pose.
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
        getClimberLeftTransform: Callable[[], Transform3d],
        getClimberRightTransform: Callable[[], Transform3d],
        getIntakeTransform: Callable[[], Transform3d],
        getTurretTransform: Callable[[], Transform3d],
        getHoodAngle: Callable[[], Rotation2d],
        period: seconds = 0.02,
    ):
        """
        Initializes the Visualizer3D with functions to get the current transforms

        :param getClimberLeftTransform: Function to get the left climber transform
        :type getClimberLeftTransform: Callable[[], Transform3d]
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
        self._getClimberLeftTransform = getClimberLeftTransform
        self._getClimberRightTransform = getClimberRightTransform
        self._getIntakeTransform = getIntakeTransform
        self._getTurretTransform = getTurretTransform
        self._getHoodAngle = getHoodAngle

        self._nettable = NetworkTableInstance.getDefault().getTable(
            "000MechVisualizer3D"
        )
        self._mechPosePub = self._nettable.getStructArrayTopic(
            "MechPoses", Pose3d
        ).publish()

        self._notifier = Notifier(self._update)
        self._notifier.startPeriodic(period)

    def _update(self) -> None:
        """
        Update the mechanism poses and publish them to the network table.
        This is a private method that should be called by self._notifier periodically.

        :return: None
        """
        mechPoses = [
            self._climberLeftInitialPose.transformBy(self._getClimberLeftTransform()),
            self._climberRightInitialPose.transformBy(self._getClimberRightTransform()),
            self._intakeInitialPose.transformBy(self._getIntakeTransform()),
            self._turretInitialPose.transformBy(self._getTurretTransform()),
            self._hoodInitialPose.rotateAround(
                self._hoodRotationTranslation, Rotation3d(self._getHoodAngle())
            ),
        ]

        self._mechPosePub.set(mechPoses)
