from math import hypot, pi

from typing import Callable

from wpilib import RobotController, RobotBase

from wpimath.geometry import Transform3d, Pose3d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import microseconds, seconds

from robotpy_apriltag import AprilTagFieldLayout

from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator

from tools.BraveLogger import BraveLogger, CameraData

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

    _poseEstimator: PhotonPoseEstimator
    """
    The pose estimator object from photonvision
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

    _data: CameraData

    def __init__(
        self,
        cameraName: str,
        apriltagFieldLayout: AprilTagFieldLayout,
        robotToCamera: Transform3d,
        logVisionMeasurement: Callable[
            [Pose3d, seconds, tuple[float, float, float] | None], None
        ],
        cameraData: CameraData,
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
        self._poseEstimator = PhotonPoseEstimator(apriltagFieldLayout, robotToCamera)
        self._logVisionMeasurement = logVisionMeasurement

        self._data = cameraData

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

    def update(
        self,
        baseConfidence: float,
        baseRotationConfidence: float,
        visionEnabled: bool = True,
        trustRotation: bool = False,
    ) -> tuple[Pose3d | None, list[int]]:
        """
        Updates the pose estimator with the latest camera results
        The Vision class is responsible for calling this periodically

        :return: The estimated robot pose and the list of seen target IDs
        :rtype: tuple[Pose3d | None, list[int]]
        """
        if not self._camera.isConnected():
            return (None, [])
        try:
            targets: list[int] = []
            result = self._camera.getLatestResult()
            poseEstimate = self._poseEstimator.estimateCoprocMultiTagPose(result)
            if poseEstimate is None:
                poseEstimate = self._poseEstimator.estimateLowestAmbiguityPose(result)

            if poseEstimate is None:
                return (None, [])

            poseEst = poseEstimate.estimatedPose
            # if poseEst.X() < 0 or poseEst.Y() < 0 or poseEst.Z() < -0.1:
            #     self._data.hasTarget = False
            #     return (None, targets)

            multitag = result.multitagResult
            tagCount = (
                len(multitag.fiducialIDsUsed)
                if multitag is not None
                else len(result.getTargets())
            )
            if tagCount == 0:
                self._data.hasTarget = False
                return (None, targets)

            avgDist = (
                sum(
                    target.bestCameraToTarget.translation().norm()
                    for target in result.getTargets()
                )
                / tagCount
            )
            stdevs = self._calculateStdDevs(
                baseConfidence,
                baseRotationConfidence,
                avgDist,
                tagCount,
                trustRotation,
            )
            self._data.hasTarget = True
            self._data.estimatedRobotPose = poseEst
            self._data.translationStdev = stdevs[0]
            self._data.rotationStdev = stdevs[2]
            if visionEnabled:
                self._logVisionMeasurement(
                    poseEst,
                    poseEstimate.timestampSeconds,
                    stdevs,
                )
            targets.extend(tag.getFiducialId() for tag in result.getTargets())

            return (poseEst, targets)
        finally:
            BraveLogger.pushSubsystemData(self._data)

    def getCameraSim(self) -> PhotonCameraSim | None:
        """
        Get the camera simulation object for this camera

        :return: The PhotonCameraSim object, or None if not in simulation
        :rtype: PhotonCameraSim | None
        """
        return self._simCamera

    def _calculateStdDevs(
        self,
        baseConfidence: float,
        baseRotationConfidence: float,
        avgTagDistance: float,
        tagCount: int,
        trustRotation: bool = False,
    ) -> tuple[float, float, float]:
        translationConfidence = baseConfidence * (avgTagDistance**2) / tagCount
        rotationConfidence = baseRotationConfidence * (avgTagDistance**2) / tagCount
        return (
            translationConfidence,
            translationConfidence,
            (float("inf") if not trustRotation else rotationConfidence),
        )

    @staticmethod
    def _transformToPose(transform: Transform3d) -> Pose3d:
        return Pose3d(
            transform.X(),
            transform.Y(),
            transform.Z(),
            transform.rotation(),
        )
