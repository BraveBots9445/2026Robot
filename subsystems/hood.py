from wpimath.units import degreesToRotations

from phoenix6.hardware import CANrange
from phoenix6.configs import (
    CANrangeConfiguration,
    ProximityParamsConfigs,
    FovParamsConfigs,
    TalonFXConfiguration,
    CurrentLimitsConfigs,
    FeedbackConfigs,
    HardwareLimitSwitchConfigs,
)
from phoenix6.signals import ForwardLimitSourceValue, ForwardLimitTypeValue

from subsystems.baseSubsystems import ServoBaseSubsystem, ServoBaseSubsystemData


class Hood:
    GEAR_RATIO: float = 36 * 16 / 170

    MAX_REVERSE_POSITION_ROTATIONS: float = degreesToRotations(73)

    def __init__(
        self, hoodMotorID: int, canrangeID: int, canbus: str = "canivore"
    ) -> None:
        self._canrange = CANrange(canrangeID, canbus)

        self._canrangeConfig = (
            CANrangeConfiguration()
            .with_proximity_params(
                ProximityParamsConfigs().with_proximity_threshold(0.05)
            )
            .with_fov_params(FovParamsConfigs().with_fov_range_x(4).with_fov_range_y(4))
        )

        motorConfig = (
            TalonFXConfiguration()
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
                .with_forward_limit_enable(True)
                .with_forward_limit_autoset_position_enable(True)
                .with_forward_limit_autoset_position_value(
                    self.MAX_REVERSE_POSITION_ROTATIONS
                )
                .with_forward_limit_source()
            )
        )
