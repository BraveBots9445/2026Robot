from math import hypot, pi

from typing import Callable

from wpilib import RobotController, RobotBase

from wpimath.geometry import Transform3d, Pose3d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import microseconds, seconds

from robotpy_apriltag import AprilTagFieldLayout

from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator

if RobotBase.isSimulation():
    from photonlibpy.simulation.photonCameraSim import PhotonCameraSim
    from photonlibpy.simulation.simCameraProperties import SimCameraProperties


class VisionCamera:
    """
    Class for a single vision camera and pose estimator used with photonvision
    """

    _camera: PhotonCamera
    """
    The camera object from photonvision
    """

    _simCamera = None
    """
    The simulated camera object for use in simulation, or None if not in simulation
    :type simCamera: PhotonCameraSim | None
    """

    _pose_estimator: PhotonPoseEstimator
    """
    The pose estimator object from photonvision
    """

    _baseStdDevs: tuple[float, float, float] = (0.1, 0.1, pi)
    """
    The default standard deviations in meters and radians to modify based on measurement factors
    """

    _jerkStdDevFactor: float = 50000
    """
    The factor to multiply the jerk by when calculating standard deviations.
    This is to penalize sudden changes in estimated pose, which are likely to be errors.
    """

    _logVisionMeasurement: Callable[
        [Pose3d, seconds, tuple[float, float, float] | None], None
    ]
    """
    A method of the drivetrain passed as a callable to be used to add vision measurement results 
    """

    _getRobotVelocity: Callable[[], ChassisSpeeds]
    """
    A method of the drivetrain passed as a clalable to be used to determine how fast the robot is moving
    """

    _offsetStore: dict[int, tuple[Transform3d, seconds]] | None
    """
    A lookup table of target ID to Transform3d offsets seen by this camera, and the FPGA timestamp they were seen at
    Will be a dict when _storeOffsets is True, otherwise None
    """
    _storeOffsets: bool = False
    """
    Whether to store offsets of seen tags for later use
    if this is False, _offsetStore will be None, otherwise, it will be a dict
    """

    _prevEst: Pose3d | None = None

    def __init__(
        self,
        cameraName: str,
        apriltagFieldLayout: AprilTagFieldLayout,
        robotToCamera: Transform3d,
        logVisionMeasurement: Callable[
            [Pose3d, seconds, tuple[float, float, float] | None], None
        ],
        getRobotVelocity: Callable[[], ChassisSpeeds],
        storeOffsets: bool = False,
        simCameraProperties=None,
    ) -> None:
        """
        Docstring for __init__

        :param cameraName: The name of the camera as configured in photonvision
        :type cameraName: str
        :param apriltagFieldLayout: The AprilTag field layout used for pose estimation
        :type apriltagFieldLayout: AprilTagFieldLayout
        :param robotToCamera: The offset from the center of the robot at the z level of the carpet to the camera in NWU order
        :type robotToCamera: Transform3d
        :param logVisionMeasurement: A callable to log vision measurements to the drivetrain and update its odometry
        :type logVisionMeasurement: Callable[[Pose3d, seconds, tuple[float, float, float] | None], None]
        :param getRobotVelocity: A callable to get the current robot velocity
        :type getRobotVelocity: Callable[[], ChassisSpeeds]
        :param storeOffsets: Whether to store the offsets of seen tags for later use
        :type storeOffsets: bool
        :param simCameraProperties: The properties of the camera to use in simulation. Value is ignored in real robot
        :type simCameraProperties: SimCameraProperties
        """
        self._camera = PhotonCamera(cameraName)
        self._pose_estimator = PhotonPoseEstimator(apriltagFieldLayout, robotToCamera)
        self._logVisionMeasurement = logVisionMeasurement
        self._getRobotVelocity = getRobotVelocity

        if storeOffsets:
            self._offsetStore = {}
        else:
            self._offsetStore = None
        self._storeOffsets = storeOffsets

        if RobotBase.isSimulation():

            # simCameraProperties = (
            #     SimCameraProperties.PERFECT_90DEG()
            # )  # use this to test perfect camera (no noise simulation)
            # the below are type ignore because the sim imports are conditional on RobotBase.isSimulation()
            # that makes them potentially unbound, but always safe to use.
            simCameraProperties = SimCameraProperties.OV9281_1280_720()  # type: ignore
            self._simCamera = PhotonCameraSim(self._camera, simCameraProperties)  # type: ignore
            self._simCamera.setMaxSightRange(5)
            # Wireframe is not implemented in python photonvision yet
            # self._simCamera.enableDrawWireframe(True)

    def update(self) -> tuple[Pose3d | None, list[int]]:
        """
        Updates the pose estimator with the latest camera results
        The Vision class is responsible for calling this periodically

        :return: The estimated robot pose and the list of seen target IDs
        :rtype: tuple[Pose3d | None, list[int]]
        """
        targets: list[int] = []
        result = self._camera.getLatestResult()
        bestTarget = result.getBestTarget()
        if bestTarget is None:
            return (None, targets)
        distance = bestTarget.getBestCameraToTarget()
        estPose = self._pose_estimator.estimateCoprocMultiTagPose(result)
        if estPose is None:
            estPose = self._pose_estimator.estimateLowestAmbiguityPose(result)
        if estPose is None:
            return (None, targets)
        poseRes = estPose.estimatedPose
        if poseRes.X() < 0 or poseRes.Y() < 0 or poseRes.Z() < -0.1:
            return (None, targets)
        self._logVisionMeasurement(
            estPose.estimatedPose,
            estPose.timestampSeconds,
            self._calculateStdDevs(
                distance, bestTarget.poseAmbiguity, estPose.estimatedPose
            ),
        )
        self._prevEst = estPose.estimatedPose
        lastPose = estPose.estimatedPose
        targets.extend(tag.getFiducialId() for tag in result.getTargets())
        if (
            self._storeOffsets and self._offsetStore is not None
        ):  # the and is for lsp typechecking
            currTime = RobotController.getFPGATime()
            for tag in result.getTargets():
                self._offsetStore[tag.getFiducialId()] = (
                    tag.getBestCameraToTarget(),
                    currTime,
                )

        return (lastPose, targets)

    def getOffset(
        self, targetID: int, timeout: microseconds = 50_000
    ) -> Transform3d | None:
        """
        Get the stored offset for a given target ID

        :param targetID: The ID of the target to get the offset for
        :type targetID: int
        :param timeout: The maximum age of the stored offset to return, in microseconds
        :type timeout: microseconds
        :return: The Transform3d offset, or None if not found (either because not seen or too old)
        :rtype: Transform3d | None
        """
        if self._offsetStore is None:
            return None
        res = self._offsetStore.get(targetID, None)
        if res is None:
            return None
        offset, timestamp = res
        if RobotController.getFPGATime() - timestamp > timeout:
            return None
        return offset

    def getCameraSim(self) -> PhotonCameraSim | None:
        """
        Get the camera simulation object for this camera

        :return: The PhotonCameraSim object, or None if not in simulation
        :rtype: PhotonCameraSim | None
        """
        return self._simCamera

    def _calculateStdDevs(
        self, distance: Transform3d, ambiguity: float, estPose: Pose3d | None = None
    ) -> tuple[float, float, float]:
        """
        Calculate standard deviations for the pose estimator based on target distance and robot velocity

        :param distance: The distance from the camera to the target in meters
        :type distance: float
        :param ambiguity: The ambiguity of the pose estimation, from 0 to 1, with 0 being no ambiguity and 1 being maximum ambiguity
        :type ambiguity: float
        :param estPose: The estimated pose of the robot, used for calculating jerk. This is optional and can be None if not available.
        :type estPose: Pose3d | None
        :return: The standard deviations for x, y, and theta
        :rtype: tuple[float, float, float]
        """
        robotSpeed = self._getRobotVelocity()
        speed = hypot(robotSpeed.vx, robotSpeed.vy)
        if speed > 4.0 or abs(robotSpeed.omega) > 3 * pi / 2:
            return (float("inf"), float("inf"), float("inf"))
        velocityFactor = 0.5 * (speed**1.5) + 0.5 * (abs(robotSpeed.omega) ** 1.5)

        distanceFactor = distance.translation().norm() ** 1.4

        # this is supposed to penalize sudden changes in estimated pose
        jerkFactor = 0
        if estPose is not None and self._prevEst is not None:
            measurementDistShift = estPose.translation().distance(
                self._prevEst.translation()
            )
            jerkFactor = (
                max(measurementDistShift, measurementDistShift**4.0)
                * self._jerkStdDevFactor
            )

        ambiguityFactor = (10 * ambiguity) ** 2

        return (
            (distanceFactor + velocityFactor + jerkFactor + ambiguityFactor)
            * self._baseStdDevs[0],
            (distanceFactor + velocityFactor + jerkFactor + ambiguityFactor)
            * self._baseStdDevs[1],
            (distanceFactor + velocityFactor + jerkFactor + ambiguityFactor)
            * self._baseStdDevs[2],
        )
