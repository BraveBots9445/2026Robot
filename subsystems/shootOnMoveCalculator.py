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
            2.1750338077545166,
            2.38704514503479,
            2.69954252243042,
            2.945054054260254,
            3.30900239944458,
            3.520456314086914,
            3.7732982635498047,
            4.097684860229492,
            4.460996627807617,
            4.932258605957031,
            5.404358386993408,
            5.770778179168701,
            5.958467960357666,
            6.401236534118652,
            7.007723808288574,
            7.396358966827393,
            7.396366119384766,
            7.943430423736572,
            8.602522850036621,
            9.316576957702637,
            9.872834205627441,
            10.316035270690918,
            10.813508033752441,
            11.126900672912598,
            11.797362327575684,
        ]
    )

    _hoodAngleInterpArray = array(
        [
            52.35594940185547,
            62.0,
            62.0,
            62.0,
            51.54261016845703,
            50.43833923339844,
            50.946048736572266,
            48.946048736572266,
            48.946048736572266,
            48.946048736572266,
            47.94601821899414,
            47.94601821899414,
            47.60641860961914,
            49.18086624145508,
            49.18086624145508,
            46.18086624145508,
            49.18086624145508,
            45.48492431640625,
            43.783287048339844,
            40.783287048339844,
            40.783287048339844,
            40.783287048339844,
            39.449405670166016,
            37.46284103393555,
            35.13908004760742,
        ]
    )

    _flywheelVelInterpArray = array(
        [
            2876.870361328125,
            2842.76123046875,
            2842.76123046875,
            2842.76123046875,
            3129.064697265625,
            3221.01025390625,
            3280.28857421875,
            3366.03369140625,
            3416.03369140625,
            3531.032470703125,
            3689.138916015625,
            3789.63916015625,
            3791.28515625,
            4034.69140625,
            4331.4560546875,
            4427.11083984375,
            4427.11083984375,
            4510.6552734375,
            4560.6552734375,
            4772.66650390625,
            4954.89208984375,
            5070.21533203125,
            5121.77978515625,
            5373.9853515625,
            5724.91455078125,
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
        self._tmpFinalPosePub = self._nettable.getStructTopic(
            "Robot Pose Translated", Pose3d
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

        t = dist / (v0 * cos(degreesToRadians(angleDeg)))
        # print(dist, flywheelRpm, v0, cos(degreesToRadians(angleDeg)), t)

        return (
            StateSetpoint(
                # 0,
                # Rotation2d(),
                flywheelRpm,
                Rotation2d.fromDegrees(angleDeg),
                angleOff - robotPose.rotation().toRotation2d(),
            ),
            t,
        )

    def getSetpoints(self, target: Pose3d, maxIterations: int = 3) -> StateSetpoint:
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
        virtualTarget = target.transformBy(self._launcherTransform)
        setpoints = StateSetpoint(0, Rotation2d(), Rotation2d())
        robotVel = self._getRobotVelocity()
        prevTimeOfFlight = float("inf")
        for _ in range(maxIterations):
            setpoints, timeToShot = self._getSetpointsStep(virtualTarget)
            virtualTarget = virtualTarget.transformBy(
                self._ChassisSpeedsToTranslation3d(robotVel, timeToShot).inverse()
            )
            if timeToShot - prevTimeOfFlight <= 0.15:
                break
            prevTimeOfFlight = timeToShot
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
