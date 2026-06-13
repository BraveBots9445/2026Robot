from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor, LinearSystemId

from phoenix6.configs import (
    TalonFXConfiguration,
    CurrentLimitsConfigs,
    MotorOutputConfigs,
)
from phoenix6.signals import NeutralModeValue, InvertedValue

from subsystems.baseSubsystems import RollerBaseSubsystem, RollerBaseSubsystemData


class Indexer(RollerBaseSubsystem):
    GEAR_RATIO: float = 1.0 / 1.0

    def __init__(self, motorID: int, canbus: str, enabled: bool = True) -> None:
        motorConfig = (
            TalonFXConfiguration()
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(30)
                .with_stator_current_limit_enable(True)
            )
            .with_motor_output(
                MotorOutputConfigs()
                .with_neutral_mode(NeutralModeValue.COAST)
                .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            )
        )

        simObject = FlywheelSim(
            LinearSystemId.flywheelSystem(
                DCMotor.krakenX60(), 0.000001, 1 / self.GEAR_RATIO
            ),
            DCMotor.krakenX60(),
        )

        super().__init__(
            "Indexer",
            motorID,
            motorConfig,
            simObject,
            canbus,
            motorToMechanismRatio=self.GEAR_RATIO,
            enabled=enabled,
        )

    def spin(self) -> None:
        self.setSetpoint(1.0)

    def stop(self) -> None:
        self.setSetpoint(0)
