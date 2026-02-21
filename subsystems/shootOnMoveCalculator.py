from math import cos, atan2, sqrt

from typing import Callable

from numpy import array, interp

from dataclasses import dataclass

from ntcore import NetworkTableInstance

from wpimath.geometry import Transform3d, Rotation2d, Pose3d, Rotation3d, Translation3d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import (
    revolutions_per_minute,
    seconds,
    meters_per_second,
    inches,
    inchesToMeters,
    metersToInches,
    meters,
    degreesToRadians,
)
from wpilib import SmartDashboard


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

    _flywheelRpmToMuzzleVelocity: Callable[[meters_per_second], revolutions_per_minute]
    """
    A function that converts flywheel RPM to muzzle velocity. 
    """

    _distanceInterpArray = array(
        [
            2.074866533279419,
            2.3398525714874268,
            2.6280407905578613,
            2.9307923316955566,
            3.2976644039154053,
            3.7175345420837402,
            4.117430210113525,
            4.922016620635986,
            5.3604326248168945,
            5.802000999450684,
            6.25164794921875,
            6.707947254180908,
            7.169476509094238,
            7.627717971801758,
            8.08549690246582,
            8.54677677154541,
            9.005876541137695,
            9.467438697814941,
        ]
    )

    _hoodAngleInterpArray = array(
        [
            15.0,
            16.235946655273438,
            18.77642250061035,
            21.151641845703125,
            21.151641845703125,
            27.272762298583984,
            28.704532623291016,
            29.14328384399414,
            34.07639694213867,
            35.492027282714844,
            36.555419921875,
            37.51203918457031,
            34.4100341796875,
            34.4100341796875,
            38.919307708740234,
            41.036529541015625,
            41.036529541015625,
            39.285911560058594,
        ]
    )

    _flywheelVelInterpArray = array(
        [
            1548.8006591796875,
            1618.1334228515625,
            1721.47509765625,
            1757.922119140625,
            1885.3424072265625,
            1885.3424072265625,
            1957.6324462890625,
            2137.9013671875,
            2137.9013671875,
            2221.438232421875,
            2322.739501953125,
            2385.3427734375,
            2430.968505859375,
            2512.1025390625,
            2558.818115234375,
            2598.804931640625,
            2698.445068359375,
            2767.07080078125,
        ]
    )

    def __init__(
        self,
        getRobotPose: Callable[[], Pose3d],
        getRobotVelocity: Callable[[], ChassisSpeeds],
        launcherTransform: Transform3d,
        flywheelRpmToMuzzleVelocity: Callable[
            [revolutions_per_minute], meters_per_second
        ],
    ) -> None:
        """
        Initializes a new ShootOnMoveCalculator.

        :param getRobotPose: A function that returns the current robot pose.
        :type getRobotPose: Callable[[], Pose3d]
        :param getRobotVelocity: A function that returns the current robot velocity in the field oriented reference frame.
        :type getRobotVelocity: Callable[[], ChassisSpeeds]
        :param launcherTransform: A transform representing the position and orientation of the launcher relative to the robot center. This should not include turret angle
        :type launcherTransform: Transform3d
        :param flywheelRpmToMuzzleVelocity: A function that converts flywheel RPM to muzzle velocity.
        :type flywheelRpmToMuzzleVelocity: Callable[[revolutions_per_minute], meters_per_second]
        """
        self._getRobotPose = getRobotPose
        self._getRobotVelocity = getRobotVelocity
        self._launcherTransform = launcherTransform
        self._flywheelRpmToMuzzleVelocity = flywheelRpmToMuzzleVelocity

        self._nettable = NetworkTableInstance.getDefault().getTable(
            "ShootOnMoveCalculator"
        )
        self._targetPub = self._nettable.getStructTopic("Target", Pose3d).publish()
        self._virtualTargetPub = self._nettable.getStructTopic(
            "VirtualTarget", Pose3d
        ).publish()

    def _getSetpointsStep(self, target: Pose3d) -> tuple[StateSetpoint, seconds]:
        """
        Calculates the setpoints for the current robot state and a target pose.

        :param target: The target pose to shoot at.
        :type target: Pose3d
        :return: A tuple containing the StateSetpoint and the time until the shot should be taken.
        :rtype: tuple[StateSetpoint, seconds]
        """
        # Get current robot state
        robotPose = self._getRobotPose()
        dist = (
            robotPose.translation()
            .toTranslation2d()
            .distance(target.translation().toTranslation2d())
        )

        displacement = target.relativeTo(robotPose)
        turretAngleRads = atan2(displacement.Y(), displacement.X())

        angleDeg = interp(dist, self._distanceInterpArray, self._hoodAngleInterpArray)
        flywheelRpm = interp(
            dist, self._distanceInterpArray, self._flywheelVelInterpArray
        )
        v0 = self._flywheelRpmToMuzzleVelocity(flywheelRpm)

        t = dist / (v0 * cos(degreesToRadians(angleDeg)))
        # print(dist, flywheelRpm, v0, cos(degreesToRadians(angleDeg)), t)

        return (
            StateSetpoint(
                flywheelRpm,
                Rotation2d.fromDegrees(angleDeg),
                Rotation2d(turretAngleRads),
            ),
            t,
        )

    def getSetpoints(self, target: Pose3d, iterations: int = 3) -> StateSetpoint:
        """
        Get the setpoints for the shooter, hood, and turret
        Uses a Recursive LuT method with a moving virtual target to account for the moving robot

        :param target: The target pose to shoot at.
        :type target: Pose3d
        :param iterations: The number of times to repeat the recursive LuT process. More iterations will result in more accurate setpoints, but will take more time to calculate. Defaults to 3. Minimum is 1
        :type iterations: int, optional
        """
        self._targetPub.set(target)
        if iterations < 1:
            iterations = 1
        virtualTarget = target.transformBy(self._launcherTransform)
        setpoints = StateSetpoint(0, Rotation2d(), Rotation2d())
        robotVel = self._getRobotVelocity()
        for _ in range(iterations):
            setpoints, timeToShot = self._getSetpointsStep(virtualTarget)
            virtualTarget = virtualTarget.transformBy(
                self._ChassisSpeedsToTranslation3d(robotVel, timeToShot).inverse()
            )
        self._virtualTargetPub.set(virtualTarget)
        return setpoints

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
