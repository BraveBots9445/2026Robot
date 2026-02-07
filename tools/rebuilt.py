from typing import ClassVar, Final
from ntcore import NetworkTableInstance, StructPublisher
from wpilib import DriverStation
from wpimath.geometry import *
from wpimath.units import *


class Rebuilt:
    Length: ClassVar[meters] = inchesToMeters(651.2)
    Width: ClassVar[meters] = inchesToMeters(317.7)

    Translation: Translation3d = Translation3d(Length / 2, Width / 2, 0)
    Rotation: Rotation3d = Rotation3d.fromDegrees(0.0, 0.0, 180.0)

    @staticmethod
    def getPosition(position: RebuiltPositions) -> Pose3d:
        pose: Pose3d = position
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            pose = pose.rotateAround(Rebuilt.Translation, Rebuilt.Rotation)
        return pose

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
        Translation3d(inchesToMeters(18), inchesToMeters(18), 0.0), Rotation3d()
    )
    PassRight: Final[RebuiltPositions] = Pose3d(
        Translation3d(inchesToMeters(18), Rebuilt.Width - inchesToMeters(18), 0.0),
        Rotation3d(),
    )
