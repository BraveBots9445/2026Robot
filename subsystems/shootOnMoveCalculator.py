from math import atan2, acos, cos

from typing import Callable

from dataclasses import dataclass

import bisect

from wpimath.geometry import Transform3d, Rotation2d, Pose3d, Rotation3d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import revolutions_per_minute, seconds, meters_per_second


@dataclass
class StateSetpoint:
    """
    A setpoint for the shooter state.
    """

    flywheelRpm: revolutions_per_minute
    """
    The flywheel RPM setpoint.
    """

    hoodAngle: Rotation2d
    """
    The hood angle setpoint.
    """

    turretAngle: Rotation2d
    """
    The turret angle setpoint.
    """


class _InterpolatingMap:
    """
    An interpolating map for looking up values based on keys with linear interpolation.
    """

    _keys: list[float]
    """
    The keys in the map.
    """

    _values: list[float]
    """
    The values in the map.
    """

    def __init__(self, keys: list[float] = [], values: list[float] = []) -> None:
        self._keys = keys
        self._values = values

    def put(self, key: float, value: float) -> None:
        i = bisect.bisect_left(self._keys, key)
        if i < len(self._keys) and self._keys[i] == key:
            self._values[i] = value
        else:
            self._keys.insert(i, key)
            self._values.insert(i, value)

    def get(self, key: float) -> float | None:
        if not self._keys:
            return None
        i = bisect.bisect_left(self._keys, key)
        if i == 0:
            return self._values[0]
        if i == len(self._keys):
            return self._values[-1]
        key0 = self._keys[i - 1]
        key1 = self._keys[i]
        value0 = self._values[i - 1]
        value1 = self._values[i]
        t = (key - key0) / (key1 - key0)
        return value0 + t * (value1 - value0)


class ShootOnMoveCalculator:
    """
    The class do to the inverse kinematics calculations for shooting while moving.
    This does not consider velocity in z axis (up and down).
    """

    _getRobotPose: Callable[[], Pose3d]
    """
    A function that returns the current robot pose.
    """

    _getRobotVelocity: Callable[[], ChassisSpeeds]
    """
    A function that returns the current robot velocity in the field oriented reference frame.
    """

    _launcherTransform: Transform3d
    """
    A transform representing the position and orientation of the launcher relative to the robot center.
    This does not include turret angle
    """

    _muzzleVelocityToFlywheelRpm: Callable[[meters_per_second], revolutions_per_minute]
    """
    A function that converts desired muzzle velocity to flywheel RPM.
    """

    _minMuzzleVelocity: meters_per_second
    """
    The minimum muzzle velocity of the shooter.
    """

    _maxMuzzleVelocity: meters_per_second
    """
    The maximum muzzle velocity of the shooter.
    """

    _minHoodAngle: Rotation2d
    """ 
    The minimum hood angle of the shooter.
    """

    _maxHoodAngle: Rotation2d
    """
    The maximum hood angle of the shooter.
    """

    # TODO: Tune this lookup table to the robot and shooter
    _timeLookup: _InterpolatingMap = _InterpolatingMap(keys=[0, 30], values=[0, 3])
    """
    A lookup table for time to shoot based on distance to target.
    """

    def __init__(
        self,
        getRobotPose: Callable[[], Pose3d],
        getRobotVelocity: Callable[[], ChassisSpeeds],
        launcherTransform: Transform3d,
        muzzleVelocityToFlywheelRpm: Callable[
            [meters_per_second], revolutions_per_minute
        ],
        minMuzzleVelocity: meters_per_second,
        maxMuzzleVelocity: meters_per_second,
        minHoodAngle: Rotation2d,
        maxHoodAngle: Rotation2d,
    ) -> None:
        """
        Initializes a new ShootOnMoveCalculator.

        :param getRobotPose: A function that returns the current robot pose.
        :type getRobotPose: Callable[[], Pose3d]
        :param getRobotVelocity: A function that returns the current robot velocity in the field oriented reference frame.
        :type getRobotVelocity: Callable[[], ChassisSpeeds]
        :param launcherTransform: A transform representing the position and orientation of the launcher relative to the robot center. This should not include turret angle
        :type launcherTransform: Transform3d
        :param muzzleVelocityToFlywheelRpm: A function that converts muzzle velocity to flywheel RPM.
        :type muzzleVelocityToFlywheelRpm: Callable[[meters_per_second], revolutions_per_minute]
        :param minMuzzleVelocity: The minimum muzzle velocity of the shooter.
        :type minMuzzleVelocity: meters_per_second
        :param maxMuzzleVelocity: The maximum muzzle velocity of the shooter.
        :type maxMuzzleVelocity: meters_per_second
        :param minHoodAngle: The minimum hood angle of the shooter.
        :type minHoodAngle: Rotation2d
        :param maxHoodAngle: The maximum hood angle of the shooter.
        :type maxHoodAngle: Rotation2d
        """
        self._getRobotPose = getRobotPose
        self._getRobotVelocity = getRobotVelocity
        self._launcherTransform = launcherTransform
        self._muzzleVelocityToFlywheelRpm = muzzleVelocityToFlywheelRpm
        self._minMuzzleVelocity = minMuzzleVelocity
        self._maxMuzzleVelocity = maxMuzzleVelocity
        self._minHoodAngle = minHoodAngle
        self._maxHoodAngle = maxHoodAngle

    def getSetpoints(self, targetPose: Pose3d) -> StateSetpoint | None:
        """
        Gets the shooter setpoints for shooting at the given target pose.

        :param targetPose: The target pose to shoot at.
        :type targetPose: Pose3d
        :return: The shooter setpoints, or None if no valid setpoints could be calculated.
        """

        launcherPose = self._getRobotPose() + self._launcherTransform
        robotVelocity = self._getRobotVelocity()
        targetTransform = targetPose.relativeTo(launcherPose)
        timeOfFlight = self._timeLookup.get(targetTransform.translation().norm())
        if timeOfFlight is None:
            return None
        # imagine the robot as static - the hub will be at this position relative to the robot when the ball arrives
        motionTransform = self._ChassisSpeedsToTranslation3d(
            robotVelocity, timeOfFlight
        )
        # apply the transform twice to get into a steady state of where the target will be
        targetTransform = targetTransform.transformBy(motionTransform.inverse())
        distanceToTarget = targetTransform.translation().norm()
        targetTransform = targetTransform.transformBy(motionTransform)
        timeOfFlight2 = self._timeLookup.get(distanceToTarget)
        if timeOfFlight2 is not None:
            motionTransform = self._ChassisSpeedsToTranslation3d(
                robotVelocity, timeOfFlight2
            )
            targetTransform = targetTransform.transformBy(motionTransform.inverse())
            timeOfFlight = timeOfFlight2

        turretAngle = atan2(targetTransform.Y(), targetTransform.X())
        # return StateSetpoint(0, Rotation2d(), Rotation2d(turretAngle))

        # Hood Solver v1: we want to maximize velocity for high, arcing shots
        k = targetTransform.translation().toTranslation2d().norm() / timeOfFlight
        if k > self._maxMuzzleVelocity:
            return None  # no feasible solution

        theta_cap = acos(min(1.0, k / self._maxMuzzleVelocity))
        theta_star = min(self._maxHoodAngle.radians(), theta_cap)

        if theta_star < self._minHoodAngle.radians():
            return None  # no feasible solution

        v0_star = k / cos(theta_star)
        return StateSetpoint(
            self._muzzleVelocityToFlywheelRpm(v0_star),
            Rotation2d(theta_star),
            Rotation2d(turretAngle),
        )

    def _ChassisSpeedsToTranslation3d(
        self, speeds: ChassisSpeeds, time: seconds
    ) -> Transform3d:
        """
        Converts chassis speeds to a translation 3d.

        :param speeds: The chassis speeds.
        :type speeds: ChassisSpeeds
        :return: The translation 3d.
        """

        return Transform3d(
            speeds.vx * time,
            speeds.vy * time,
            0.0,
            Rotation3d(0.0, 0.0, speeds.omega * time),
        )
