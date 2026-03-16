from math import cos, atan2, sqrt

from typing import Callable

from numpy import array, interp

from dataclasses import dataclass

from ntcore import NetworkTableInstance
from ntcore.util import ntproperty

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
    radians_per_second,
)
from wpilib import SmartDashboard, RobotController


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

    _setpoints: tuple[StateSetpoint, int]

    _distanceInterpArray = array(
        [
            3.9031262397766113,
            4.55928897857666,
            5.402111530303955,
            5.43321084976196,
            6.06634521484375,
            7.002396583557129,
            7.488589763641357,
            8.346280097961426,
        ]
    )
    _hoodAngleInterpArray = array(
        [
            62.0,
            46.94731903076172,
            46.94731903076172,
            62.0,
            46.94731903076172,
            46.94731903076172,
            47.114784240722656,
            45.0670280456543,
        ]
    )
    _flywheelVelInterpArray = array(
        [
            3441.86962890625,
            3477.87646484375,
            3706.87158203125,
            4077.8178710937,
            3937.236083984375,
            4170.0302734375,
            4594.1455078125,
            4802.18603515625,
        ]
    )

    _tofFudgeFactor = ntproperty("/ShootOnMoveCalculator/tofFudgeFactor", 0.5)

    def __init__(
        self,
        getRobotPose: Callable[[], Pose3d],
        getRobotVelocity: Callable[[], ChassisSpeeds],
        flywheelRpmToMuzzleVelocity: Callable[
            [revolutions_per_minute], meters_per_second
        ],
        launcherTransform: Transform3d = Transform3d(
            inchesToMeters(0), inchesToMeters(6), inchesToMeters(15), Rotation3d()
        ),
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

        self._setpoints = (StateSetpoint(0, Rotation2d(), Rotation2d()), -500)

        self._nettable = NetworkTableInstance.getDefault().getTable(
            "ShootOnMoveCalculator"
        )
        self._targetPub = self._nettable.getStructTopic("Target", Pose3d).publish()
        self._virtualTargetPub = self._nettable.getStructTopic(
            "VirtualTarget", Pose3d
        ).publish()
        self._tmpFinalPosePub = self._nettable.getStructTopic(
            "Robot Pose Translated", Pose3d
        ).publish()
        self._tofEstPub = self._nettable.getFloatTopic(
            "Estimated Time Of Flight s"
        ).publish()

    def _getSetpointsStep(
        self, target: Pose3d, robotOmega: radians_per_second = 0
    ) -> tuple[StateSetpoint, seconds]:
        """
        Calculates the setpoints for the current robot state and a target pose.

        :param target: The target pose to shoot at.
        :type target: Pose3d
        :return: A tuple containing the StateSetpoint and the time until the shot should be taken.
        :rtype: tuple[StateSetpoint, seconds]
        """
        # Get current robot state
        robotPose = self._getRobotPose()
        robotPose.transformBy(self._launcherTransform)
        self._tmpFinalPosePub.set(robotPose)
        dist = (
            robotPose.translation()
            .toTranslation2d()
            .distance(target.translation().toTranslation2d())
        )

        poseTranslation = robotPose.translation()
        targetTranslation = target.translation()
        angleOff = (targetTranslation - poseTranslation).toTranslation2d().angle()

        angleDeg = interp(dist, self._distanceInterpArray, self._hoodAngleInterpArray)
        flywheelRpm = interp(
            dist, self._distanceInterpArray, self._flywheelVelInterpArray
        )
        v0 = self._flywheelRpmToMuzzleVelocity(flywheelRpm)

        t = dist / (v0 * cos(degreesToRadians(angleDeg))) * self._tofFudgeFactor
        # print(dist, flywheelRpm, v0, cos(degreesToRadians(angleDeg)), t)

        return (
            StateSetpoint(
                # 0,
                # Rotation2d(),
                flywheelRpm,
                Rotation2d.fromDegrees(angleDeg),
                angleOff
                - robotPose.rotation().toRotation2d()
                - Rotation2d(robotOmega * 0.02),
            ),
            t,
        )

    def _getSetpoints(self, target: Pose3d, maxIterations: int = 10) -> StateSetpoint:
        """
        Get the setpoints for the shooter, hood, and turret
        Uses a Recursive LuT method with a moving virtual target to account for the moving robot

        :param target: The target pose to shoot at.
        :type target: Pose3d
        :param iterations: The number of times to repeat the recursive LuT process. More iterations will result in more accurate setpoints, but will take more time to calculate. Defaults to 3. Minimum is 1
        :type iterations: int, optional
        """
        self._targetPub.set(target)
        setpoints, timeToShot = self._getSetpointsStep(target)
        if maxIterations < 1:
            maxIterations = 1
        # virtualTarget = target.transformBy(self._launcherTransform)
        virtualTarget = target
        setpoints = StateSetpoint(0, Rotation2d(), Rotation2d())
        robotVel = self._getRobotVelocity()
        prevTimeOfFlight = float("inf")
        for _ in range(maxIterations):
            setpoints, timeToShot = self._getSetpointsStep(virtualTarget)
            virtualTarget = target.transformBy(
                self._ChassisSpeedsToTranslation3d(robotVel, timeToShot)  # .inverse()
            )
            if timeToShot - prevTimeOfFlight <= 0.05:
                self._tofEstPub.set(timeToShot)
                break
            prevTimeOfFlight = timeToShot
        else:
            pass
            # self._nettable.putNumber("Iterations", )
        self._virtualTargetPub.set(virtualTarget)
        self._setpoints = (setpoints, RobotController.getFPGATime())
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

    def getSetpoints(
        self, target: Pose3d, maxIterations: int = 10, maxTime: seconds = 0.02
    ) -> StateSetpoint:
        if RobotController.getFPGATime() - self._setpoints[1] / (10**6) > maxTime:
            self._getSetpoints(target, maxIterations)

        return self._setpoints[0]
