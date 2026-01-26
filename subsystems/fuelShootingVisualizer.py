from math import pi

from typing import Callable

from commands2 import Subsystem, Command

from ntcore import NetworkTable, NetworkTableInstance, StructArrayPublisher

from wpimath.geometry import Pose3d, Translation3d, Rotation3d, Rotation2d, Transform3d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import revolutions_per_minute, meters


class FuelShootingVisualizer(Subsystem):
    """
    A class to visualize the fuel being shot from the robot based on the states of the subsystems
    """

    _fuel: list[tuple[Pose3d, Translation3d]] = []
    """
    A list of the fuel being managed by the vizualizer. Each fuel is represented as a tuple containing
    the current pose of the fuel as a Pose3d and the velocity of the fuel as a Translation3d. 
    """

    _getRobotPose: Callable[[], Pose3d]
    """
    A function to get the current pose of the robot.
    """

    _getRobotVelocity: Callable[[], ChassisSpeeds]
    """
    A function to get the current velocity of the robot.
    """

    _getTurretAngle: Callable[[], Rotation2d]
    """
    A function to get the current angle of the turret.
    Rotation2d(0) should be pointing towards the front of the robot.
    """

    _getHoodAngle: Callable[[], Rotation2d]
    """
    A function to get the current angle of the hood.
    This should be the exit angle of the fuel from the shooter.
    """

    _getShooterSpeed: Callable[[], revolutions_per_minute]
    """
    A function to get the current speed of the shooter flywheel in RPM.
    """

    _flywheelRadius: meters
    """
    The radius of the shooter flywheel in meters.
    """

    _robotToTurret: Transform3d
    """
    The transform from the center of the robot to the center of the turret.
    The rotation should probably be empty
    """

    _nettable: NetworkTable
    """
    The networktable used for logging to push the fuel data to the dashboard.
    """

    _fuelPosePub: StructArrayPublisher
    """
    A publisher for the fuel poses to be sent to the dashboard.
    It publishes in [Pose3d]
    """

    _kEnergyTransferEfficiency: float = 0.7
    """
    A unitless value for how much energy is transferred from the flywheel to the fuel.
    """

    def __init__(
        self,
        getRobotPose: Callable[[], Pose3d],
        getRobotVelocity: Callable[[], ChassisSpeeds],
        getTurretAngle: Callable[[], Rotation2d],
        getHoodAngle: Callable[[], Rotation2d],
        getShooterSpeed: Callable[[], revolutions_per_minute],
        flywheelRadius: meters,
        robotToTurret: Transform3d,
    ):
        """
        Initializes the FuelShootingVisualizer subsystem.
        :param getRobotPose: A function to get the current pose of the robot.
        :param getRobotVelocity: A function to get the current velocity of the robot.
        :param getTurretAngle: A function to get the current angle of the turret as the angle from the front of the robot NWU.
        :param getHoodAngle: A function to get the current angle of the hood as the exit angle of a fuel
        :param getShooterSpeed: A function to get the current speed of the shooter flywheel in RPM.
        :param flywheelRadius: The radius of the shooter flywheel in meters.
        :param robotToTurret: The transform from the center of the robot to the center of the turret. The rotation should probably be empty. It should not consider the angle of the turret about its axis
        """
        self._getRobotPose = getRobotPose
        self._getRobotVelocity = getRobotVelocity
        self._getTurretAngle = getTurretAngle
        self._getHoodAngle = getHoodAngle
        self._getShooterSpeed = getShooterSpeed
        self._flywheelRadius = flywheelRadius
        self._robotToTurret = robotToTurret

        self._nettable = NetworkTableInstance.getDefault().getTable("00FuelVisualizer")

        self._fuelPosePub = self._nettable.getStructArrayTopic(
            "FuelPoses", Pose3d
        ).publish()

    def periodic(self) -> None:
        """
        Periodic method to update the fuel positions and publish them to the dashboard.
        """
        toRemove: list[int] = []
        for idx, (pose, velocity) in enumerate(self._fuel):
            if pose.Z() < -5:
                toRemove.append(idx)
                continue
            self._fuel[idx] = (
                Pose3d(
                    pose.X() + velocity.X() * 0.02,
                    pose.Y() + velocity.Y() * 0.02,
                    pose.Z() + velocity.Z() * 0.02,
                    Rotation3d(),
                ),
                Translation3d(
                    velocity.X(),
                    velocity.Y(),
                    velocity.Z() - 9.8 * 0.02,
                ),
            )

        for idx in reversed(toRemove):
            self._fuel.pop(idx)

        fuel_poses = [pose for pose, _ in self._fuel]
        self._fuelPosePub.set(fuel_poses)

    def launch(self) -> None:
        """
        Launches a fuel from the robot based on the current states of the subsystems.
        """
        robotPose = self._getRobotPose()
        robotVelocity = self._getRobotVelocity()
        muzzleVelocity = (
            self._getShooterSpeed()
            * self._flywheelRadius
            * 2
            * pi
            / 60
            * self._kEnergyTransferEfficiency
        )
        self._fuel.append(
            (
                robotPose + self._robotToTurret,
                Translation3d(
                    robotVelocity.vx
                    + muzzleVelocity
                    * self._getHoodAngle().cos()
                    * self._getTurretAngle().cos(),
                    robotVelocity.vy
                    + muzzleVelocity
                    * self._getHoodAngle().cos()
                    * self._getTurretAngle().sin(),
                    muzzleVelocity * self._getHoodAngle().sin(),
                ),
            )
        )

    def launchCommand(self) -> Command:
        """
        Creates a command to launch a fuel when executed.

        :return: A command to launch a fuel
        :rtype: Command
        """
        return self.runOnce(self.launch)
