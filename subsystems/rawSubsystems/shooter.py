from enum import Enum

from dataclasses import dataclass

from wpilib.simulation import FlywheelSim
from wpimath.system.plant import LinearSystemId, DCMotor
from wpimath.units import meters
from wpiutil.wpistruct import make_wpistruct

from phoenix6.configs import TalonFXConfiguration, Slot0Configs, CurrentLimitsConfigs
from phoenix6.units import rotations_per_second

import numpy as np

from subsystems.baseSubsystems.flywheelBaseSubsystem import (
    FlywheelBaseSubsystem,
    FlywheelBaseSubsystemData,
)


class ShooterInterpolationTable:
    SHOOTING_TABLE = (
        np.array([0.0, 1.0, 1.5, 2.0]),
        np.array([2000.0, 2200.0, 2500.0, 3000.0]),
    )

    PASSING_TABLE = (
        np.array([0.0, 1.0, 1.5, 2.0]),
        np.array([2000.0, 2200.0, 2500.0, 3000.0]),
    )


@make_wpistruct
@dataclass
class ShooterData:
    masterMotor: FlywheelBaseSubsystemData
    slaveMotor: FlywheelBaseSubsystemData


class Shooter:
    GEAR_RATIO: float = 1.0 / 1.0
    IDLE_VELOCITY_RPS: float = 65

    def __init__(
        self,
        flywheelMasterID: int,
        flywheelSlaveID: int,
        canbus: str = "canivore",
        enabled: bool = True,
    ):
        flywheelConfig = (
            TalonFXConfiguration()
            .with_slot0(
                Slot0Configs()
                .with_k_p(0.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.01007)
                .with_k_a(0.0)
            )
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(40)
                .with_stator_current_limit_enable(True)
            )
        )

        simObject = FlywheelSim(
            LinearSystemId.flywheelSystem(
                DCMotor.krakenX60(2), 0.0001, 1 / self.GEAR_RATIO
            ),
            DCMotor.krakenX60(2),
        )

        self._flywheelMaster = FlywheelBaseSubsystem(
            "Shooter Flywheel Master",
            flywheelMasterID,
            flywheelConfig,
            simObject,
            canbus,
            enabled=enabled,
        )

        self._flywheelSlave = FlywheelBaseSubsystem(
            "Shooter Flywheel Slave",
            flywheelSlaveID,
            flywheelConfig,
            simObject,
            canbus,
        )

        self._flywheelMaster.addSlave(self._flywheelSlave, True)

    def periodic(self) -> ShooterData:
        return ShooterData(
            self._flywheelMaster.periodic(), self._flywheelSlave.periodic()
        )

    def simulationPeriodic(self) -> None:
        self._flywheelMaster.simulationPeriodic()
        self._flywheelSlave.simulationPeriodic()

    def setSetpoint(self, setpoint: rotations_per_second) -> None:
        self._flywheelMaster.setSetpoint(setpoint)

    def atSetpoint(self, tolerance: float = 2.0) -> bool:
        return self._flywheelMaster.atSetpoint(tolerance)

    def interpolate(
        self,
        distance: meters,
        table: ShooterInterpolationTable = ShooterInterpolationTable.SHOOTING_TABLE,  # type: ignore
    ) -> None:
        self.setSetpoint(np.interp(distance, table[0], table[1]))  # type: ignore
