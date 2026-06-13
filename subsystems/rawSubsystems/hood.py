from wpimath.units import degreesToRotations, meters, rotationsToRadians
from wpimath.system.plant import DCMotor
from wpilib.simulation import SingleJointedArmSim
from wpimath.geometry import Rotation2d
from wpilib import RobotBase

from phoenix6.hardware import CANrange
from phoenix6.configs import (
    CANrangeConfiguration,
    ProximityParamsConfigs,
    FovParamsConfigs,
    TalonFXConfiguration,
    CurrentLimitsConfigs,
    FeedbackConfigs,
    HardwareLimitSwitchConfigs,
    Slot0Configs,
)
from phoenix6.signals import ForwardLimitSourceValue

import numpy as np

from subsystems.baseSubsystems import ServoBaseSubsystem, ServoBaseSubsystemData


class HoodInterpolationTable:
    SHOOTING_TABLE = (
        np.array([0.0, 1.0, 1.5, 2.0]),
        np.array([70.0, 70.5, 71.0, 71.5]),
    )

    PASSING_TABLE = (
        np.array([0.0, 1.0, 1.5, 2.0]),
        np.array([70.0, 70.5, 71.0, 71.5]),
    )


class Hood:
    GEAR_RATIO: float = 36 * 16 / 170

    MAX_REVERSE_POSITION_ROTATIONS: float = degreesToRotations(70)
    MAX_FORWARD_POSITION_ROTATIONS: float = degreesToRotations(73)

    def __init__(
        self,
        hoodMotorID: int,
        canrangeID: int,
        canbus: str = "canivore",
        enabled: bool = True,
    ) -> None:
        self._canrange = CANrange(canrangeID, canbus)

        self._canrangeConfig = (
            CANrangeConfiguration()
            .with_proximity_params(
                ProximityParamsConfigs().with_proximity_threshold(0.05)
            )
            .with_fov_params(FovParamsConfigs().with_fov_range_x(4).with_fov_range_y(4))
        )

        self._canrange.configurator.apply(self._canrangeConfig)

        motorConfig = (
            TalonFXConfiguration()
            .with_slot0(
                Slot0Configs()
                .with_k_p(0.25)
                .with_k_i(0.0)
                .with_k_d(0.1)
                .with_k_s(0)
                .with_k_v(0)
                .with_k_a(0)
            )
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(35)
                .with_stator_current_limit_enable(True)
            )
            .with_feedback(
                FeedbackConfigs().with_sensor_to_mechanism_ratio(self.GEAR_RATIO)
            )
            .with_hardware_limit_switch(
                HardwareLimitSwitchConfigs()
                .with_forward_limit_remote_sensor_id(canrangeID)
                .with_forward_limit_source(ForwardLimitSourceValue.REMOTE_CANRANGE)
                .with_forward_limit_enable(RobotBase.isReal())
                .with_forward_limit_autoset_position_enable(True)
                .with_forward_limit_autoset_position_value(
                    self.MAX_FORWARD_POSITION_ROTATIONS
                )
            )
        )

        simObject = SingleJointedArmSim(
            DCMotor.krakenX60(),
            1 / self.GEAR_RATIO,
            0.001,
            0.001,
            -float("inf"),
            float("inf"),
            False,
            self.MAX_FORWARD_POSITION_ROTATIONS,
        )

        self._servo = ServoBaseSubsystem(
            "Hood",
            hoodMotorID,
            motorConfig,
            simObject,
            canbus,
            motorToMechanismRatio=self.GEAR_RATIO,
            enabled=enabled,
        )

        self._servo.setSetpoint(self.MAX_FORWARD_POSITION_ROTATIONS)

    def periodic(self) -> ServoBaseSubsystemData:
        return self._servo.periodic()

    def simulationPeriodic(self) -> None:
        self._servo.simulationPeriodic()
        self._canrange.sim_state.set_distance(5)

    def setAngleSetpointRotations(self, setpointRotations: float) -> None:
        self._servo.setSetpoint(setpointRotations)

    def setAngleSetpointDegrees(self, setpointDegrees: float) -> None:
        self.setAngleSetpointRotations(degreesToRotations(setpointDegrees))

    def setAngleSetpointRotation2d(self, setpoint: Rotation2d) -> None:
        self.setAngleSetpointDegrees(setpoint.degrees())

    def duck(self) -> None:
        self.setAngleSetpointRotations(self.MAX_FORWARD_POSITION_ROTATIONS)

    def atSetpoint(self) -> bool:
        return self._servo.atSetpoint()

    def interpolate(
        self,
        distance: meters,
        table: HoodInterpolationTable = HoodInterpolationTable.SHOOTING_TABLE,  # type: ignore
    ) -> None:
        self.setAngleSetpointDegrees(np.interp(distance, table[0], table[1]))  # type: ignore
