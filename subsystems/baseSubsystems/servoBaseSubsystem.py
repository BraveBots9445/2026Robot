from dataclasses import dataclass

from wpilib.simulation import ElevatorSim, SingleJointedArmSim, DCMotorSim

from wpiutil.wpistruct import make_wpistruct

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.status_signal import StatusSignal
from phoenix6.controls import PositionDutyCycle


class MechanismPosition(float):
    """
    A wrapper for float
    This is the position of the mechanism in its units as set in the gear ratio
    Will be mechanism rotations unless converted with the gear ratio
    """

    ...


@make_wpistruct
@dataclass
class ServoBaseSubsystemData:
    position: float
    rotorPosition: float
    statorCurrent: float
    motorDutyCycle: float
    motorTemp: float
    motorRef: float
    motorSlot: int
    setpoint: float
    enabled: bool


class ServoBaseSubsystem:
    def __init__(
        self,
        name: str,
        motorID: int,
        motorConfig: TalonFXConfiguration,
        simObject: ElevatorSim | SingleJointedArmSim,
        canBus: str,
        *,
        cancoderID: int | None = None,
        cancoderConfigs: CANcoderConfiguration | None = None,
        motorToMechanismRatio: float = 1.0 / 1.0,
        dt: float = 0.02,
        enabled: bool = True
    ):
        self._dt = dt
        self._name = name
        self._motor = TalonFX(motorID, canBus)
        self._motorConfigs = motorConfig
        self._motorSimState = self._motor.sim_state
        self._motor.configurator.apply(self._motorConfigs)

        if cancoderID and cancoderConfigs:
            self._cancoder = CANcoder(cancoderID, canBus)
            self._cancoderConfigs = cancoderConfigs
            self._cancoder.configurator.apply(self._cancoderConfigs)
            self._motor.set_position(self._cancoder.get_position().value)
        elif ((not cancoderID) or (not cancoderConfigs)) and not (
            (not cancoderID) and (not cancoderConfigs)
        ):
            raise ValueError("Either provide both a cancoder and a config or neither")

        self._simObj = simObject

        self._motorToMechRatio = motorToMechanismRatio

        self._positionSignal = self._motor.get_position(True)
        self._rotorPositionSignal = self._motor.get_rotor_position(False)
        self._statorCurrentSignal = self._motor.get_stator_current(False)
        self._motorDutyCycleSignal = self._motor.get_duty_cycle(False)
        self._motorTempSignal = self._motor.get_device_temp(False)
        self._motorRefSignal = self._motor.get_closed_loop_reference(False)
        self._motorSlotSignal = self._motor.get_closed_loop_slot(False)

        self._setpoint = self._positionSignal.value

        self._enabled = enabled

        self._slaves: list[ServoBaseSubsystem] = []

    def periodic(self) -> ServoBaseSubsystemData:
        StatusSignal.refresh_all(
            self._positionSignal,
            self._rotorPositionSignal,
            self._statorCurrentSignal,
            self._motorDutyCycleSignal,
            self._motorTempSignal,
            self._motorRefSignal,
            self._motorSlotSignal,
        )

        if self._enabled:
            self._motor.set_control(PositionDutyCycle(self._setpoint))
        else:
            self._motor.stopMotor()

        return ServoBaseSubsystemData(
            self._positionSignal.value,
            self._rotorPositionSignal.value,
            self._statorCurrentSignal.value,
            self._motorDutyCycleSignal.value,
            self._motorTempSignal.value,
            self._motorRefSignal.value,
            self._motorSlotSignal.value,
            self._setpoint,
            self._enabled,
        )

    def simulationPeriodic(self) -> None:
        self._simObj.setInputVoltage(self._motor.get_motor_voltage().value)

        self._simObj.update(self._dt)

        motorVel = self._simObj.getVelocity() / self._motorToMechRatio
        self._motorSimState.set_rotor_velocity(motorVel)
        self._motorSimState.add_rotor_position(motorVel * self._dt)

    def setSetpoint(self, setpoint: MechanismPosition | float):
        self._setpoint = setpoint
        if self._slaves:
            for slave in self._slaves:
                slave.setSetpoint(setpoint)

    def getSetpoint(self) -> MechanismPosition:
        return self._setpoint  # type: ignore

    def atSetpoint(self, tolerance: MechanismPosition | float = 0.05) -> bool:
        return abs(self._setpoint - self._positionSignal.value) < tolerance

    def getPosition(self) -> MechanismPosition | float:
        return self._positionSignal.value

    def setEnabled(self, enabled: bool) -> None:
        self._enabled = enabled
        if self._slaves:
            for slave in self._slaves:
                slave.setEnabled(enabled)

    def disable(self) -> None:
        self.setEnabled(False)

    def enable(self) -> None:
        self.setEnabled(True)

    def isEnabled(self) -> bool:
        return self._enabled

    def addSlave(self, slave: ServoBaseSubsystem):
        """
        It is the responsibility of the caller to properly invert motors that should be
        inverted in the motor's configurations prior to calling this method
        This method handles setting enabled values for slaves
        If a CANCoder is used, only the master should pass it in to the constructor with a configuration
        """
        self._slaves.append(slave)
        slave.setEnabled(self._enabled)
