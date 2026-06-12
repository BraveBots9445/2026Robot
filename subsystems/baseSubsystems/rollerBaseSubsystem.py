from dataclasses import dataclass

from wpilib.simulation import FlywheelSim

from wpimath.units import radiansToRotations

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.status_signal import StatusSignal


class MechanismVelocity(float):
    """
    A wrapper for float
    This is the position of the mechanism in its units as set in the gear ratio
    Will be mechanism rotations unless converted with the gear ratio
    """

    ...


@dataclass
class RollerBaseSubsystemData:
    name: str
    velocity: float
    rotorVelocity: float
    statorCurrent: float
    motorDutyCycle: float
    motorTemp: float
    setpoint: float
    enabled: bool


class RollerBaseSubsystem:
    def __init__(
        self,
        name: str,
        motorID: int,
        motorConfig: TalonFXConfiguration,
        simObject: FlywheelSim,
        canbus: str,
        *,
        motorToMechanismRatio: float = 1.0 / 1.0,
        dt: float = 0.02,
        enabled: bool = True
    ):
        self._dt = dt
        self._name = name
        self._motor = TalonFX(motorID, canbus)
        self._motorConfigs = motorConfig
        self._motor.configurator.apply(self._motorConfigs)

        self._motorSimState = self._motor.sim_state

        self._simObj = simObject

        self._motorToMechRatio = motorToMechanismRatio

        self._velocitySignal = self._motor.get_velocity(True)
        self._rotorVelocitySignal = self._motor.get_rotor_velocity(False)
        self._statorCurrentSignal = self._motor.get_stator_current(False)
        self._motorDutyCycleSignal = self._motor.get_duty_cycle(False)
        self._motorTempSignal = self._motor.get_device_temp(False)
        self._motorRefSignal = self._motor.get_closed_loop_reference(False)
        self._motorSlotSignal = self._motor.get_closed_loop_slot(False)

        self._setpoint = 0

        self._enabled = enabled

        self._slaves: list[RollerBaseSubsystem] = []

    def periodic(self) -> RollerBaseSubsystemData:
        StatusSignal.refresh_all(
            self._velocitySignal,
            self._rotorVelocitySignal,
            self._statorCurrentSignal,
            self._motorDutyCycleSignal,
            self._motorTempSignal,
            self._motorRefSignal,
            self._motorSlotSignal,
        )

        if (
            type(self) == RollerBaseSubsystem
        ):  # check that this is not a `FlywheelBaseSubsystem`
            if self._enabled:
                self._motor.set(self._setpoint)
            else:
                self._motor.stopMotor()

        return RollerBaseSubsystemData(
            self._name,
            self._velocitySignal.value,
            self._rotorVelocitySignal.value,
            self._statorCurrentSignal.value,
            self._motorDutyCycleSignal.value,
            self._motorTempSignal.value,
            self._setpoint,
            self._enabled,
        )

    def simulationPeriodic(self) -> None:
        self._simObj.setInputVoltage(self._motor.get_motor_voltage().value)

        self._simObj.update(self._dt)

        motorVel = (
            radiansToRotations(self._simObj.getAngularVelocity())
            / self._motorToMechRatio
        )
        self._motorSimState.set_rotor_velocity(motorVel)
        self._motorSimState.add_rotor_position(motorVel * self._dt)

    def setSetpoint(self, setpoint: MechanismVelocity | float):
        self._setpoint = setpoint

    def getSetpoint(self) -> MechanismVelocity:
        return self._setpoint  # type: ignore

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

    def addSlave(self, slave: RollerBaseSubsystem) -> None:
        """
        It is the responsibility of the caller to properly invert motors that should be
        inverted in the motor's configurations prior to calling this method
        This method handles setting enabled values for slaves
        If a CANCoder is used, it should be passed to the master and all slaves with the same configuration
        """
        self._slaves.append(slave)
        slave.setEnabled(self._enabled)
