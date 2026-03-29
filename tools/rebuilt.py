from enum import Enum, auto, unique
from typing import ClassVar, Final
from ntcore import NetworkTableInstance, StructPublisher
from wpilib import DriverStation
from wpimath.geometry import *
from wpimath.units import *


class Rebuilt:
    @unique
    class Zones(Enum):
        HUB = auto()
        PASSLEFT = auto()
        PASSRIGHT = auto()

    Length: ClassVar[meters] = inchesToMeters(651.2)
    Width: ClassVar[meters] = inchesToMeters(317.7)

    Translation: Translation3d = Translation3d(Length / 2, Width / 2, 0)
    Rotation: Rotation3d = Rotation3d.fromDegrees(0.0, 0.0, 180.0)

    @staticmethod
    def getPosition(position: RebuiltPositions | Pose3d ) -> Pose3d:
        pose: Pose3d = position
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            pose = pose.rotateAround(Rebuilt.Translation, Rebuilt.Rotation)
        return pose

    @staticmethod
    def getZone(position: Pose2d | Pose3d) -> Rebuilt.Zones | None:
        position = position if isinstance( position, Pose3d ) else Pose3d( position )
        fieldPos: Translation2d = Rebuilt.getPosition( position ).translation().toTranslation2d()
        
        if RebuiltZones.Hub.contains( fieldPos ):
            return Rebuilt.Zones.HUB
        if RebuiltZones.PassLeft.contains( fieldPos ):
            return Rebuilt.Zones.PASSLEFT
        if RebuiltZones.PassRight.contains( fieldPos ):
            return Rebuilt.Zones.PASSRIGHT
        
        return None

    @staticmethod
    def publishPositions() -> list[StructPublisher]:
        tbl = NetworkTableInstance.getDefault().getTable("RebuiltPositions")
        publishers: list = []
        for attr in dir(RebuiltPositions):
            if not attr.startswith("__"):
                topic = tbl.getStructTopic(attr, Pose3d).publish()
                topic.set(
                    Rebuilt.getPosition(
                        RebuiltPositions.__getattribute__(RebuiltPositions, attr)
                    )
                )
                publishers.append(topic)
        return publishers


class RebuiltPositions:
    Hub: Final[RebuiltPositions] = Pose3d(
        Translation3d(
            inchesToMeters(158.6 + 47.0 / 2),
            inchesToMeters(317.7 / 2),
            inchesToMeters(72.0),
        ),
        Rotation3d(),
    )
    PassLeft: Final[RebuiltPositions] = Pose3d(
        Translation3d(
            inchesToMeters(48), Rebuilt.Width - inchesToMeters(78), inchesToMeters(20.0)
        ),
        Rotation3d(),
    )
    PassRight: Final[RebuiltPositions] = Pose3d(
        Translation3d(inchesToMeters(48), inchesToMeters(78), inchesToMeters(20.0)),
        Rotation3d(),
    )


class RebuiltZones:
    Hub = Rectangle2d(
        Translation2d(0.0, 0.0),
        Translation2d( RebuiltPositions.Hub.translation().X(), Rebuilt.Width )
    )
    PassLeft = Rectangle2d(
        RebuiltPositions.Hub.translation().toTranslation2d(),
        Translation2d( Rebuilt.Length, Rebuilt.Width )
    )
    PassRight = Rectangle2d(
        RebuiltPositions.Hub.translation().toTranslation2d(),
        Translation2d( Rebuilt.Length, 0 )
    )
