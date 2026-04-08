from typing import Callable

from commands2 import Command, InstantCommand
from wpilib import RobotBase, SmartDashboard
from math import e, pi, hypot
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

import threading
import time

from wpilib import RobotBase, Notifier, RobotState


from .visionCamera import VisionCamera

from tools.BraveLogger import (
    ShooterLeftCameraData,
    ShooterRightCameraData,
)


class Vision:
    _enabled = ntproperty("000Vision/Enabled", True)

    # these names and their associated positions are fake
    _turretCamera: VisionCamera
    _shooterRightCamera: VisionCamera
    # _backLeftReverseCamera: VisionCamera
    _shooterLeftCamera: VisionCamera

    # TODO: The below offsets are all garbage from copilot
    _shooterLeftRobotToCamera: Transform3d = Transform3d(
        Translation3d(inchesToMeters(-3.5), inchesToMeters(9.5), inchesToMeters(16.75)),
        Rotation3d.fromDegrees(0, 9.0584, -155),
    )

    _shooterRightCameraToRobot: Transform3d = Transform3d(
        Translation3d(
            inchesToMeters(-3.5), inchesToMeters(-9.5), inchesToMeters(16.75)
        ),
        Rotation3d.fromDegrees(0, 9.0584, 155),
    )

    # _backLeftReverseCameraToRobot: Transform3d = Transform3d(
    #     Translation3d(
    #         inchesToMeters(-12.5), inchesToMeters(13.5), inchesToMeters(7.75)
    #     ),
    #     Rotation3d.fromDegrees(0, 30 + 5.6 if RobotBase.isReal() else 0, 120),
    # )

    # _backRightForwardCameraToRobot: Transform3d = Transform3d(
    #     Translation3d(
    #         inchesToMeters(-10.5), inchesToMeters(-13.5), inchesToMeters(7.75)
    #     ),
    #     Rotation3d.fromDegrees(0, 30 + 2.04 if RobotBase.isReal() else 0, -60),
    # )

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
            [Pose3d, seconds, tuple[float, float, float] | None], None
        ],
        getRobotVelocity: Callable[[], ChassisSpeeds],
        getRobotPose: Callable[[], Pose2d],
    ):
        """
        Construct the Vision subsystem

        :param logVisionMeasurement: A callable to add vision measurement results to the drivetrain
        :type logVisionMeasurement: Callable[[Pose3d, seconds, tuple[float, float, float] | None], None]
        :param getRobotVelocity: A callable to get the current robot velocity
        :type getRobotVelocity: Callable[[], ChassisSpeeds]
        :param getRobotPose: A callable to get the current robot pose for simulation purposes only
        :type getRobotPose: Callable[[], Pose2d]
        """
        self.nettable = NetworkTableInstance.getDefault().getTable("000Vision")

        self._getRobotVelocity = getRobotVelocity

        self._shooterRightCamera = VisionCamera(
            "ArducamOV9281-ShooterRight",
            self._tagLayout,
            self._shooterRightCameraToRobot,
            logVisionMeasurement,
            ShooterRightCameraData(False, 0, 0, Pose3d()),
        )

        self._shooterLeftCamera = VisionCamera(
            "ArducamOV9281-ShooterLeft",
            self._tagLayout,
            self._shooterLeftRobotToCamera,
            logVisionMeasurement,
            ShooterLeftCameraData(False, 0, 0, Pose3d()),
        )

        # self._backRightForwardCamera = VisionCamera(
        #     "ArducamOV9281-BR-F",
        #     self._tagLayout,
        #     self._backRightForwardCameraToRobot,
        #     logVisionMeasurement,
        #     getRobotVelocity,
        # )

        # self._backLeftReverseCamera = VisionCamera(
        #     "ArducamOV9281-BL-R",
        #     self._tagLayout,
        #     self._backLeftReverseCameraToRobot,
        #     logVisionMeasurement,
        #     getRobotVelocity,
        # )

        self._poseEstPub = self.nettable.getStructArrayTopic(
            "EstimatedPoses",
            Pose2d,
        ).publish()

        self._detectedTagsPub = self.nettable.getStructArrayTopic(
            "DetectedTags",
            Pose3d,
        ).publish()

        if RobotBase.isSimulation():
            from photonlibpy.simulation import visionSystemSim

            self._getRobotPose = getRobotPose
            self._visionSim = visionSystemSim.VisionSystemSim("photonvisionSim")
            self._visionSim.addAprilTags(self._tagLayout)
            self._visionSim.addCamera(
                self._shooterLeftCamera.getCameraSim(), self._shooterLeftRobotToCamera  # type: ignore
            )
            # self._visionSim.addCamera(
            #     self._backLeftReverseCamera.getCameraSim(), self._backLeftReverseCameraToRobot  # type: ignore
            # )
            # self._visionSim.addCamera(
            #     self._backRightForwardCamera.getCameraSim(), self._backRightForwardCameraToRobot  # type: ignore
            # )
            self._visionSim.addCamera(
                self._shooterRightCamera.getCameraSim(), self._shooterRightCameraToRobot  # type: ignore
            )
            # SmartDashboard.putData(self._visionSim.getDebugField())
            self._simNotifier = Notifier(self._simulationPeriodic)
            self._simNotifier.startPeriodic(0.02)
        self._periodicRunning = False
        self._visionUpdatePeriod = 0.05
        threading.Thread(
            target=self._visionLoop, daemon=True, name="Vision-periodic"
        ).start()

    def _visionLoop(self) -> None:
        """Daemon thread loop: runs _periodic, skipping if a previous run is still active."""
        while True:
            time.sleep(self._visionUpdatePeriod)
            if not self._periodicRunning:
                self._periodicRunning = True
                try:
                    self._periodic()
                finally:
                    self._periodicRunning = False

    def _periodic(self) -> None:
        # turret camera does not do pose estimation
        enabled = self._enabled
        if (
            not self._enabled
        ):  # or (RobotState.isAutonomous() and RobotState.isEnabled()):
            enabled = False

        vel = self._getRobotVelocity()
        speed = hypot(vel.vx, vel.vy)
        if speed > 2.75 or abs(vel.omega) > degreesToRadians(180):
            enabled = False

        tags = []
        trustRotation = speed < 0.5 and vel.omega_dps < 15
        baseConfidence = 0.2
        rotationConfidence = 0.3 if RobotState.isDisabled() else 10
        # _, BLRTags = self._backLeftReverseCamera.update()
        _, shooterRightTags = self._shooterRightCamera.update(
            baseConfidence, rotationConfidence, enabled, trustRotation
        )
        tags.extend(shooterRightTags)
        # _, BRFTags = self._backRightForwardCamera.update()
        # if not tags:
        _, ShooterLeftTags = self._shooterLeftCamera.update(
            baseConfidence * (len(tags) + 1),
            rotationConfidence * (len(tags) + 1),
            enabled,
            trustRotation,
        )
        tags.extend(ShooterLeftTags)

        self._detectedTagsPub.set([self._tagLayout.getTagPose(tag) for tag in tags])

    def _simulationPeriodic(self) -> None:
        """
        This is not simulationPeriodic, but _simulationPeriodic so the command scheduler does not get to it and we can run it in a different thread
        """
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
        return InstantCommand(self.toggleEnabled)

    def enableCommand(self) -> Command:
        """
        Get a command that enables vision processing

        :return: A command that enables vision processing
        :rtype: Command
        """
        return InstantCommand(lambda: self.setEnabled(True))

    def disableCommand(self) -> Command:
        """
        Get a command that disables vision processing

        :return: A command that disables vision processing
        :rtype: Command
        """
        return InstantCommand(lambda: self.setEnabled(False))

    @staticmethod
    def _pose3dToPose2d(pose3d: Pose3d) -> Pose2d:
        """
        Convert a Pose3d to a Pose2d by dropping the z component and converting rotation

        :param pose3d: The Pose3d to convert
        :type pose3d: Pose3d
        :return: The converted Pose2d
        :rtype: Pose2d
        """
        return Pose2d(pose3d.X(), pose3d.Y(), pose3d.rotation().toRotation2d())
