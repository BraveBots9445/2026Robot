from typing import Callable

from commands2 import Command, InstantCommand, Subsystem
from wpilib import RobotBase, SmartDashboard
from math import e, pi
from ntcore import NetworkTableInstance, StructArrayPublisher
from ntcore.util import ntproperty

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.units import inchesToMeters, degreesToRadians
from wpimath.geometry import (
    Pose2d,
    Translation3d,
    Transform3d,
    Rotation3d,
    Pose3d,
    Rotation2d,
)
from wpimath.units import (
    seconds,
    meters,
    radians,
    meters_per_second,
    degrees_per_second,
)
from wpimath.kinematics import ChassisSpeeds

from wpilib import RobotBase

from photonlibpy.simulation import visionSystemSim

from .visionCamera import VisionCamera


class Vision(Subsystem):
    _enabled = ntproperty("000Vision/Enabled", True)

    # these names and their associated positions are fake
    _turretCamera: VisionCamera
    _frontLeftCamera: VisionCamera
    _frontRightCamera: VisionCamera
    _rearCamera: VisionCamera

    # TODO: The below offsets are all garbage from copilot
    _turretCameraToRobot: Transform3d = Transform3d(
        Translation3d(inchesToMeters(0), inchesToMeters(0), inchesToMeters(10)),
        Rotation3d(0, 0, 0),
    )

    _frontLeftCameraToRobot: Transform3d = Transform3d(
        Translation3d(inchesToMeters(10), inchesToMeters(10), inchesToMeters(10)),
        Rotation3d.fromDegrees(0, 0, 45),
    )

    _frontRightCameraToRobot: Transform3d = Transform3d(
        Translation3d(inchesToMeters(10), inchesToMeters(-10), inchesToMeters(10)),
        Rotation3d.fromDegrees(0, 0, -45),
    )

    _rearCameraToRobot: Transform3d = Transform3d(
        Translation3d(inchesToMeters(-10), inchesToMeters(0), inchesToMeters(10)),
        Rotation3d.fromDegrees(0, 0, 180),
    )

    _tagLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(
        # AprilTagField.kDefaultField
        AprilTagField.k2026RebuiltWelded
    )

    _getRobotPose: Callable[[], Pose2d] | None

    _poseEstPub: StructArrayPublisher
    """
    A publisher for the estimated positions from each camera to be sent to the dashboard/advantagescope
    sends list[Pose2d]
    """

    _detectedTagsPub: StructArrayPublisher
    """
    A publisher to send the list of currently detected tags to the dashboard/advantagescope 
    sends list[Pose3d]
    """

    def __init__(
        self,
        logVisionMeasurement: Callable[
            [Pose3d, int, tuple[float, float, float] | None], None
        ],
        getRobotVelocity: Callable[[], ChassisSpeeds],
        getRobotPose: Callable[[], Pose2d],
    ):
        """
        Construct the Vision subsystem

        :param logVisionMeasurement: A callable to add vision measurement results to the drivetrain
        :type logVisionMeasurement: Callable[[Pose2d, int, tuple[float, float, float] | None], None]
        :param getRobotVelocity: A callable to get the current robot velocity
        :type getRobotVelocity: Callable[[], ChassisSpeeds]
        :param getRobotPose: A callable to get the current robot pose for simulation purposes only
        :type getRobotPose: Callable[[], Pose2d]
        """
        self.nettable = NetworkTableInstance.getDefault().getTable("000Vision")

        self._turretCamera = VisionCamera(
            "TurretCamera",
            self._tagLayout,
            self._turretCameraToRobot,
            lambda _arg1, _arg2, _arg3: None,  # the turret never does pose estimation - it just tracks targets
            lambda: ChassisSpeeds(0, 0, 0),
        )

        self._frontLeftCamera = VisionCamera(
            "FrontLeftCamera",
            self._tagLayout,
            self._frontLeftCameraToRobot,
            logVisionMeasurement,
            getRobotVelocity,
        )

        self._frontRightCamera = VisionCamera(
            "FrontRightCamera",
            self._tagLayout,
            self._frontRightCameraToRobot,
            logVisionMeasurement,
            getRobotVelocity,
        )

        self._rearCamera = VisionCamera(
            "RearCamera",
            self._tagLayout,
            self._rearCameraToRobot,
            logVisionMeasurement,
            getRobotVelocity,
        )

        self._poseEstPub = self.nettable.getStructArrayTopic(
            "EstimatedPoses",
            Pose2d,
        ).publish()

        self._detectedTagsPub = self.nettable.getStructArrayTopic(
            "DetectedTags",
            Pose3d,
        ).publish()

        if RobotBase.isSimulation():
            self._getRobotPose = getRobotPose
            self._visionSim = visionSystemSim.VisionSystemSim("photonvisionSim")
            self._visionSim.addAprilTags(self._tagLayout)
            self._visionSim.addCamera(self._turretCamera.getCameraSim(), self._turretCameraToRobot)  # type: ignore
            self._visionSim.addCamera(
                self._frontLeftCamera.getCameraSim(), self._frontLeftCameraToRobot  # type: ignore
            )
            self._visionSim.addCamera(
                self._frontRightCamera.getCameraSim(), self._frontRightCameraToRobot  # type: ignore
            )
            self._visionSim.addCamera(self._rearCamera.getCameraSim(), self._rearCameraToRobot)  # type: ignore
            SmartDashboard.putData(self._visionSim.getDebugField())

    def periodic(self) -> None:
        # turret camera does not do pose estimation

        if not self._enabled:
            return
        estFL, tagsFL = self._frontLeftCamera.update()
        estFR, tagsFR = self._frontRightCamera.update()
        estR, tagsR = self._rearCamera.update()
        _estTu, tagsTu = self._turretCamera.update()

        self._poseEstPub.set(
            []
            + ([self._pose3dToPose2d(estFL)] if estFL is not None else [])
            + ([self._pose3dToPose2d(estFR)] if estFR is not None else [])
            + ([self._pose3dToPose2d(estR)] if estR is not None else [])
        )
        self._detectedTagsPub.set(
            list(
                self._tagLayout.getTagPose(tag)
                for tag in tagsFL + tagsFR + tagsR + tagsTu
            )
        )

    def simulationPeriodic(self) -> None:
        # self._getRobotPose should never be None in simulation, so type: ignore is safe
        self._visionSim.update(self._getRobotPose())  # type: ignore

    def setEnabled(self, enabled: bool) -> None:
        """
        Enable or disable vision processing

        :param enabled: Whether vision processing should be enabled
        :type enabled: bool
        """
        self._enabled = enabled

    def toggleEnabled(self) -> None:
        """
        Toggle whether vision processing is enabled
        """
        self._enabled = not self._enabled

    def isEnabled(self) -> bool:
        """
        Check whether vision processing is enabled
        :return: True if vision processing is enabled, False otherwise
        """
        return self._enabled

    def toggleEnabledCommand(self) -> Command:
        """
        Get a command that toggles whether vision processing is enabled

        :return: A command that toggles vision processing
        :rtype: Command
        """
        return InstantCommand(self.toggleEnabled, self)

    def enableCommand(self) -> Command:
        """
        Get a command that enables vision processing

        :return: A command that enables vision processing
        :rtype: Command
        """
        return InstantCommand(lambda: self.setEnabled(True), self)

    def disableCommand(self) -> Command:
        """
        Get a command that disables vision processing

        :return: A command that disables vision processing
        :rtype: Command
        """
        return InstantCommand(lambda: self.setEnabled(False), self)

    def _pose3dToPose2d(self, pose3d: Pose3d) -> Pose2d:
        """
        Convert a Pose3d to a Pose2d by dropping the z component and converting rotation

        :param pose3d: The Pose3d to convert
        :type pose3d: Pose3d
        :return: The converted Pose2d
        :rtype: Pose2d
        """
        return Pose2d(pose3d.X(), pose3d.Y(), pose3d.rotation().toRotation2d())
