from dataclasses import dataclass

from wpilib.simulation import SingleJointedArmSim, FlywheelSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import inchesToMeters, degreesToRadians

from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
    Slot0Configs,
    FeedbackConfigs,
    CurrentLimitsConfigs,
    MagnetSensorConfigs,
    MotorOutputConfigs,
)
from phoenix6.signals import (
    GravityTypeValue,
    SensorDirectionValue,
    NeutralModeValue,
    InvertedValue,
)


from subsystems.baseSubsystems import (
    ServoBaseSubsystem,
    RollerBaseSubsystem,
    ServoBaseSubsystemData,
    RollerBaseSubsystemData,
)


@dataclass
class IntakeData:
    pivotData: ServoBaseSubsystemData
    pivotSlaveData: ServoBaseSubsystemData
    rollerData: RollerBaseSubsystemData


class Intake:
    PIVOT_GEAR_RATIO: float = 15.0 / 1.0
    ROLLER_GEAR_RATIO: float = 15.0 / 1.0  # I don't remember, but this feels possible

    INTAKE_INTAKING_POSITION: float = 0
    INTAKE_RETRACT_POSITION: float = 0.375
    INTAKE_AGITATE_LOW: float = 0.05
    INTAKE_AGITATE_MID: float = 0.125
    INTAKE_AGITATE_HIGH: float = 0.25

    ROLLER_VELOCITY: float = 0.6

    def __init__(
        self,
        pivotMasterID: int,
        pivotSlaveID: int,
        canCoderID: int,
        rollerID: int,
        canbus: str = "canivore",
        enabled: bool = True,
    ) -> None:
        pivotMasterInverted = False
        pivotConfig = (
            TalonFXConfiguration()
            .with_slot0(
                Slot0Configs()
                .with_k_p(0.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_g(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
                .with_gravity_type(GravityTypeValue.ARM_COSINE)
            )
            .with_feedback(FeedbackConfigs().with_feedback_remote_sensor_id(canCoderID))
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(40)
                .with_stator_current_limit_enable(True)
            )
            .with_motor_output(
                MotorOutputConfigs()
                .with_neutral_mode(NeutralModeValue.BRAKE)
                .with_inverted(
                    InvertedValue.CLOCKWISE_POSITIVE
                    if pivotMasterInverted
                    else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                )
            )
        )

        rollerConfig = TalonFXConfiguration().with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(50)
            .with_stator_current_limit_enable(True)
        )

        cancoderConfig = CANcoderConfiguration().with_magnet_sensor(
            MagnetSensorConfigs()
            .with_magnet_offset(0.0)
            .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
        )

        pivotSim = SingleJointedArmSim(
            DCMotor.krakenX60(2),
            self.PIVOT_GEAR_RATIO,
            0.1016,
            inchesToMeters(12.5),
            0,
            degreesToRadians(135),
            True,
            degreesToRadians(135),
        )

        rollerGearbox = DCMotor.krakenX60(1)
        rollerSim = FlywheelSim(
            LinearSystemId.flywheelSystem(rollerGearbox, 0.001, self.ROLLER_GEAR_RATIO),
            rollerGearbox,
        )

        self._pivotMaster = ServoBaseSubsystem(
            "Intake Pivot Master",
            pivotMasterID,
            pivotConfig.with_motor_output(
                pivotConfig.motor_output.with_inverted(
                    InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                    if pivotMasterInverted
                    else InvertedValue.CLOCKWISE_POSITIVE
                )
            ),
            pivotSim,
            canbus,
            cancoderID=canCoderID,
            cancoderConfigs=cancoderConfig,
            motorToMechanismRatio=self.PIVOT_GEAR_RATIO,
            enabled=enabled,
        )

        self._pivotMaster.addSlave(self._pivotSlave)

        self._pivotSlave = ServoBaseSubsystem(
            "Intake Pivot Slave",
            pivotSlaveID,
            pivotConfig,
            pivotSim,
            canbus,
            motorToMechanismRatio=self.PIVOT_GEAR_RATIO,
        )

        self._roller = RollerBaseSubsystem(
            rollerID,
            rollerConfig,
            rollerSim,
            canbus,
            motorToMechanismRatio=self.ROLLER_GEAR_RATIO,
            enabled=enabled,
        )

    def periodic(self) -> IntakeData:
        pivotData = self._pivotMaster.periodic()
        pivotSlaveData = self._pivotSlave.periodic()
        rollerData = self._roller.periodic()

        return IntakeData(pivotData, pivotSlaveData, rollerData)

    def intake(self) -> None:
        self._pivotMaster.setSetpoint(self.INTAKE_INTAKING_POSITION)
        self._roller.setSetpoint(self.ROLLER_VELOCITY)

    def retract(self) -> None:
        self._pivotMaster.setSetpoint(self.INTAKE_RETRACT_POSITION)
        self._roller.setSetpoint(0)

    def agitateLow(self) -> None:
        self._pivotMaster.setSetpoint(self.INTAKE_AGITATE_LOW)
        self._roller.setSetpoint(self.ROLLER_VELOCITY)

    def agitateMid(self) -> None:
        self._pivotMaster.setSetpoint(self.INTAKE_AGITATE_MID)
        self._roller.setSetpoint(self.ROLLER_VELOCITY)

    def agitateHigh(self) -> None:
        self._pivotMaster.setSetpoint(self.INTAKE_AGITATE_HIGH)
        self._roller.setSetpoint(self.ROLLER_VELOCITY)

    def setEnabled(self, enabled: bool) -> None:
        self._pivotMaster.setEnabled(enabled)
        self._roller.setEnabled(enabled)
