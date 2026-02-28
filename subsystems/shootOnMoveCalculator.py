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
            1.8754675388336182,
            2.028961420059204,
            2.27776837348938,
            2.5838449001312256,
            2.930488348007202,
            3.3249759674072266,
            3.7445149421691895,
            4.148898124694824,
            4.568080902099609,
            4.9903669357299805,
            5.41411828994751,
            5.840790748596191,
            6.27106237411499,
            6.747304439544678,
            7.184889316558838,
            7.6319990158081055,
            8.098421096801758,
            8.571436882019043,
            9.011550903320312,
            9.471451759338379,
            9.908583641052246,
            10.359992980957031,
        ]
    )

    _hoodAngleInterpArray = array(
        [
            50.0,
            50.0,
            50.0,
            50.0,
            50.0,
            50.0,
            40.858882904052734,
            40.858882904052734,
            36.366695404052734,
            36.366695404052734,
            36.366695404052734,
            32.554195404052734,
            28.101070404052734,
            28.134904861450195,
            28.134904861450195,
            25.952756881713867,
            34.11811065673828,
            36.71290588378906,
            31.767593383789062,
            28.72305679321289,
            25.852115631103516,
            22.180240631103516,
        ]
    )

    _flywheelVelInterpArray = array(
        [
            2968.16259765625,
            3175.818115234375,
            3318.909912109375,
            3420.524658203125,
            3605.06591796875,
            4239.05029296875,
            3972.25341796875,
            4239.05029296875,
            4272.4931640625,
            4279.6875,
            4379.6875,
            4532.421875,
            4653.90625,
            4853.5341796875,
            4861.7373046875,
            5032.8310546875,
            5182.00341796875,
            5448.40966796875,
            5448.40966796875,
            5514.42236328125,
            5821.84423828125,
            5920.30322265625,
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
