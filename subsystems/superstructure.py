from dataclasses import dataclass

from enum import Enum

from commands2 import Subsystem, Command

from ntcore import NetworkTableInstance, StructPublisher

from wpiutil.wpistruct import make_wpistruct
from wpimath.units import degreesToRotations

from subsystems.rawSubsystems import (
    Hood,
    Hopper,
    Indexer,
    Intake,
    Shooter,
    IntakeData,
    ShooterData,
)
from subsystems.baseSubsystems import (
    ServoBaseSubsystemData,
    RollerBaseSubsystemData,
)


@make_wpistruct
@dataclass
class SuperstructureData:
    hood: ServoBaseSubsystemData
    hopper: RollerBaseSubsystemData
    indexer: RollerBaseSubsystemData
    intake: IntakeData
    shooter: ShooterData
    intakeState: int
    desiredIntakeState: int
    shootingState: int
    desiredShootingState: int


class IntakeStates(Enum):
    STOWED = 0
    EXTENDED = 1
    AGITATING = 2


class ShootingStates(Enum):
    IDLE = 0
    SHOOTING_INTERP = 1  # NOT IMPLEMENTED
    PASSING_INTERP = 2  # NOT IMPLEMENTED
    SHOOTING_STATIC = 3
    PASSING_STATIC = 4


class Superstructure(Subsystem):
    CANBUS: str = "canivore1"

    def __init__(self) -> None:
        self._networktable = NetworkTableInstance.getDefault().getTable(
            "Superstructure"
        )

        self._telemPub = self._networktable.getStructTopic(
            "Telem", SuperstructureData
        ).publish()

        self._hood = Hood(
            28,
            0,
            self.CANBUS,
            enabled=True,
        )

        self._hopper = Hopper(
            24,
            self.CANBUS,
            enabled=True,
        )

        self._indexer = Indexer(
            23,
            self.CANBUS,
            enabled=True,
        )

        self._intake = Intake(
            20,
            21,
            20,
            22,
            self.CANBUS,
            enabled=True,
        )

        self._shooter = Shooter(
            26,
            27,
            self.CANBUS,
            enabled=True,
        )

        self._intakeState = IntakeStates.STOWED
        self._desiredIntakeState = self._intakeState

        self._shootingState = ShootingStates.IDLE
        self._desiredShootingState = self._shootingState

    def periodic(self) -> None:
        if self._desiredIntakeState == IntakeStates.AGITATING:
            if self._intakeState != IntakeStates.AGITATING:
                self._intake.agitateHigh()
                self._intakeState = IntakeStates.AGITATING
            if self._intake.atPivotSetpoint():
                intakePosition = degreesToRotations(
                    self._intake.getPivotPosition().degrees()
                )
                if abs(intakePosition - self._intake.INTAKE_AGITATE_HIGH) < abs(
                    intakePosition - self._intake.INTAKE_AGITATE_LOW
                ):
                    self._intake.agitateLow()
                else:
                    self._intake.agitateHigh()
        elif (
            self._desiredIntakeState == IntakeStates.STOWED
            and self._intakeState != IntakeStates.STOWED
        ):
            self._intake.stow()
            if self._intake.atPivotSetpoint():
                self._intakeState = IntakeStates.STOWED
        elif (
            self._desiredIntakeState == IntakeStates.EXTENDED
            and self._intakeState != IntakeStates.EXTENDED
        ):
            self._intake.intake()
            if self._intake.atPivotSetpoint():
                self._intakeState = IntakeStates.EXTENDED

        if (
            self._desiredShootingState == ShootingStates.IDLE
            and self._shootingState != ShootingStates.IDLE
        ):
            self._hopper.stop()
            self._indexer.stop()
            self._shooter.setSetpoint(self._shooter.IDLE_VELOCITY_RPS)
            self._hood.duck()
            if self._shooter.atSetpoint() and self._hood.atSetpoint():
                self._shootingState = ShootingStates.IDLE
        elif (
            self._desiredShootingState == ShootingStates.SHOOTING_STATIC
            and self._shootingState != ShootingStates.SHOOTING_STATIC
        ):
            self._shooter.setSetpoint(70)
            self._hood.setAngleSetpointDegrees(72)
            if self._shooter.atSetpoint() and self._hood.atSetpoint():
                self._indexer.spin()
                self._hopper.spin()
                self._shootingState = ShootingStates.SHOOTING_STATIC
        elif (
            self._desiredShootingState == ShootingStates.PASSING_STATIC
            and self._shootingState != ShootingStates.PASSING_STATIC
        ):
            self._shooter.setSetpoint(72)
            self._hood.setAngleSetpointDegrees(73)
            if self._shooter.atSetpoint() and self._hood.atSetpoint():
                self._indexer.spin()
                self._hopper.spin()
                self._shootingState = ShootingStates.PASSING_STATIC

        data = SuperstructureData(
            self._hood.periodic(),
            self._hopper.periodic(),
            self._indexer.periodic(),
            self._intake.periodic(),
            self._shooter.periodic(),
            self._intakeState.value,
            self._desiredIntakeState.value,
            self._shootingState.value,
            self._desiredShootingState.value,
        )

        self._telemPub.set(data)

    def simulationPeriodic(self) -> None:
        self._hood.simulationPeriodic()
        self._hopper.simulationPeriodic()
        self._indexer.simulationPeriodic()
        self._intake.simulationPeriodic()
        self._shooter.simulationPeriodic()

    def intakeExtend(self) -> Command:
        def do() -> None:
            self._desiredIntakeState = IntakeStates.EXTENDED

        return self.runOnce(do)

    def intakeStow(self) -> Command:
        def do() -> None:
            self._desiredIntakeState = IntakeStates.STOWED

        return self.runOnce(do)

    def intakeAgitate(self) -> Command:
        def do() -> None:
            self._desiredIntakeState = IntakeStates.AGITATING

        return self.runOnce(do)

    def shooterIdle(self) -> Command:
        def do() -> None:
            self._desiredShootingState = ShootingStates.IDLE

        return self.runOnce(do)

    def shooterShootStatic(self) -> Command:
        def do() -> None:
            self._desiredShootingState = ShootingStates.SHOOTING_STATIC

        return self.runOnce(do)

    def shooterPassStatic(self) -> Command:
        def do() -> None:
            self._desiredShootingState = ShootingStates.PASSING_STATIC

        return self.runOnce(do)
