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
from wpilib import DriverStation


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

    _shootDistanceInterpArray = array(
        [
            2.080439567565918,
            2.5823168754577637,
            2.833693742752075,
            3.051513671875,
            3.7620387077331543,
            3.7790729999542236,
            4.707864284515381,
            4.945676803588867,
            5.61357307434082,
            5.885664463043213,
            6.203237533569336,
        ]
    )

    _shootHoodAngleInterpArray = array(
        [
            68.0,
            61.45344543457031,
            58.81499481201172,
            57.92197799682617,
            46.533504486083984,
            52.69411087036133,
            45.7725715637207,
            27.69704818725586,
            42.33541488647461,
            31.657405853271484,
            28.270965576171875,
        ]
    )

    _shootFlywheelVelInterpArray = array(
        [
            2684.827880859375,
            2962.234619140625,
            3493.436279296875,
            3123.27880859375,
            3252.91845703125,
            3263.772705078125,
            3566.302001953125,
            3460.345947265625,
            3679.562255859375,
            3941.315673828125,
            4034.179443359375,
        ]
    )

    _passDistanceInterpArray = array([0, 10, 20])

    _passHoodAngleInterpArray = array([40, 20, 20])

    _passFlywheelVelInterpArray = array([4000, 4500, 4500])

    _tofFudgeFactor = ntproperty("/ShootOnMoveCalculator/tofFudgeFactor", 0.17)

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
        self, target: Pose3d, robotOmega: radians_per_second = 0, passing: bool = False
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

        if passing and len(self._passDistanceInterpArray) > 0:
            angleDeg = interp(
                dist, self._passDistanceInterpArray, self._passHoodAngleInterpArray
            )
            flywheelRpm = interp(
                dist, self._passDistanceInterpArray, self._passFlywheelVelInterpArray
            )
        else:
            if (
                dist < self._shootDistanceInterpArray[0]
                or dist > self._shootDistanceInterpArray[-1]
            ):
                angleDeg = self._backupHoodLookupClosedForm(dist).degrees()
                flywheelRpm = self._backupFlywheelLookupClosedForm(dist)
            else:
                angleDeg = interp(
                    dist,
                    self._shootDistanceInterpArray,
                    self._shootHoodAngleInterpArray,
                )
                flywheelRpm = interp(
                    dist,
                    self._shootDistanceInterpArray,
                    self._shootFlywheelVelInterpArray,
                )
        v0 = self._flywheelRpmToMuzzleVelocity(flywheelRpm)

        t = dist / (v0 * cos(degreesToRadians(angleDeg))) * self._tofFudgeFactor

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

    def _getSetpoints(
        self, target: Pose3d, maxIterations: int = 3, passing: bool = False
    ) -> StateSetpoint:
        """
        Get the setpoints for the shooter, hood, and turret
        Uses a Recursive LuT method with a moving virtual target to account for the moving robot

        :param target: The target pose to shoot at.
        :type target: Pose3d
        :param iterations: The number of times to repeat the recursive LuT process. More iterations will result in more accurate setpoints, but will take more time to calculate. Defaults to 3. Minimum is 1
        :type iterations: int, optional
        """
        self._targetPub.set(target)
        if maxIterations < 1:
            maxIterations = 1
        # virtualTarget = target.transformBy(self._launcherTransform)
        virtualTarget = target
        setpoints = StateSetpoint(0, Rotation2d(), Rotation2d())
        timeToShot = 0.0
        robotVel = self._getRobotVelocity()
        prevTimeOfFlight = float("inf")
        for _ in range(maxIterations):
            setpoints, timeToShot = self._getSetpointsStep(
                virtualTarget, passing=passing
            )
            virtualTarget = target.transformBy(
                self._ChassisSpeedsToTranslation3d(robotVel, timeToShot).inverse()
            )
            if timeToShot - prevTimeOfFlight <= 0.05:
                break
            prevTimeOfFlight = timeToShot
        else:
            pass
            # self._nettable.putNumber("Iterations", )
        self._tofEstPub.set(timeToShot)
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
        mult = -1 if DriverStation.getAlliance() == DriverStation.Alliance.kRed else 1

        return Transform3d(
            speeds.vx * time * mult,
            speeds.vy * time * mult,
            0.0,
            Rotation3d(0.0, 0.0, speeds.omega * time),
        )

    """
    https://www.desmos.com/calculator/yh6givh54x
    desmos for linreg below
    """

    def _backupHoodLookupClosedForm(self, dist: meters) -> Rotation2d:
        # based on a linear regression of already taken data
        # 84.* is the original b, but I bumped it to 80 because we were undershooting but also too high
        return Rotation2d.fromDegrees(-9.04769 * dist + 80.75677)

    def _backupFlywheelLookupClosedForm(self, dist: meters) -> revolutions_per_minute:
        # based on a linear regression of already taken data
        # 258.* is the original a, but I bumped it to 278.
        return 278.44584 * dist + 2336.55292

    def getSetpoints(self, target: Pose3d, passing: bool = False) -> StateSetpoint:
        return self._getSetpoints(target, maxIterations=3, passing=passing)
