"""
This is the parent class for both the woahval and the indexer, since they are both just open loop motors that feed into each other.
"""

from threading import Lock

from commands2 import Subsystem

from ntcore import NetworkTable

from wpimath.system.plant import DCMotor
from wpimath.units import amperes, radiansToRotations

from phoenix6.configs import (
    TalonFXConfiguration,
    OpenLoopRampsConfigs,
    CurrentLimitsConfigs,
    MotorOutputConfigs,
)
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from phoenix6.sim import TalonFXSimState
from phoenix6.status_signal import StatusSignal
from phoenix6.units import rotations_per_second

from .BraveLogger import BraveLogger, OpenWheelData


class OpenLoopWheel(Subsystem):
    ########## HARDWARE ##########
    _motor: TalonFX
    """
    The motor that spins the wheel.
    This is a krakenx60
    """

    _dutyCycleSetpoint: float = 0.0

    ########## CONFIGURATION ##########
    _canbus: str = ""

    _motorConfig: TalonFXConfiguration

    _motorInverted: bool = False
    """
    Whether the motor is inverted.
    CCW => False
    CW => True
    """

    _shootingDutyCyle: float = 0.5
    """
    The duty cycle to run the wheel at when shooting fuel.
    """

    _idleDutyCycle: float = 0.1
    """
    The duty cycle to run the wheel at when ready to shoot, but not actually shooting.
    """

    _currentLimit: amperes = 40

    ########## LOGGING ##########
    _nettable: NetworkTable

    _data: OpenWheelData

    _lock: Lock

    _getDutyCycleSignal: StatusSignal[float]

    _getVelocitySignal: StatusSignal[rotations_per_second]

    _getCurrentSignal: StatusSignal[amperes]

    ########## SIMULATION ##########
    _motorSimState: TalonFXSimState

    def __init__(
        self,
        motorId: int,
        name: str,
        rampTime: float = 0.5,
        inverted: bool = False,
        shootingDutyCycle: float = 0.5,
        idleDutyCycle: float = 0.1,
    ) -> None:
        self._shootingDutyCyle = shootingDutyCycle
        self._idleDutyCycle = idleDutyCycle
        self._motorInverted = inverted

        self._motor = TalonFX(motorId, self._canbus)

        self._motorConfig = (
            TalonFXConfiguration()
            .with_open_loop_ramps(
                OpenLoopRampsConfigs().with_duty_cycle_open_loop_ramp_period(rampTime)
            )
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(self._currentLimit)
                .with_stator_current_limit(True)
            )
            .with_motor_output(
                MotorOutputConfigs().with_neutral_mode(NeutralModeValue.COAST)
            )
        )

        self._data = OpenWheelData(0.0, 0.0, 0.0)

        self._getCurrentSignal = self._motor.get_stator_current(False)
        self._getVelocitySignal = self._motor.get_velocity(False)
        self._getDutyCycleSignal = self._motor.get_duty_cycle(False)
        BraveLogger.registerStatusSignal(
            [self._getCurrentSignal, self._getVelocitySignal, self._getDutyCycleSignal]
        )

        self._motorSimState = self._motor.sim_state

        self._lock = Lock()

        self.setName(name)

    def periodic(self) -> None:
        # with self._lock:
        self._data.current = self._getCurrentSignal.value_as_double
        self._data.velocity = self._getVelocitySignal.value_as_double
        self._data.dutyCycle = self._getDutyCycleSignal.value_as_double
        BraveLogger.pushSubsystemData(self._data)

        self._motor.setVoltage(
            self._dutyCycleSetpoint * (1 if not self._motorInverted else -1) * 12
        )

    def simulationPeriodic(self) -> None:
        simMotor = DCMotor.krakenX60()

        vel = radiansToRotations(self._motor.get() * simMotor.freeSpeed)

        self._motorSimState.set_rotor_velocity(vel)
        self._motorSimState.add_rotor_position(vel * 0.02)

    def setSetpoint(self, dutyCycle: float) -> None:
        """
        Sets the motor setpoint for the wheel to a percent

        :param dutyCycle: The percent to set the wheel motor to, from -1.0 to 1.0. positive is feeding into the shooter
        :type dutyCycle: float
        """
        self._dutyCycleSetpoint = dutyCycle

    def getSetpoint(self) -> float:
        """
        Gets the current motor setpoint for the wheel

        :return: The percent the spindexer motor is set to, from -1.0 to 1.0. positive is feeding into the shooter
        :rtype: float
        """
        return self._dutyCycleSetpoint

    def stop(self) -> None:
        """
        Stops the wheel from spinning
        """
        self.setSetpoint(0.0)

    def setSetpointShooting(self) -> None:
        """
        Sets the wheel to the duty cycle for shooting fuel
        """
        self.setSetpoint(self._shootingDutyCyle)

    def setSetpointIdle(self) -> None:
        """
        Sets the wheel to the duty cycle for idling while ready to shoot fuel
        """
        self.setSetpoint(self._idleDutyCycle)

    def getData(self) -> OpenWheelData:
        """
        Gets the current data for the wheel

        :return: The current data for the wheel.
        :rtype: OpenWheelData
        """
        # with self._lock:
        return self._data
