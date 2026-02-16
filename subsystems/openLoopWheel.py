"""
This is the parent class for both the woahval and the indexer, since they are both just open loop motors that feed into each other.
"""

from commands2 import Subsystem

from ntcore import NetworkTable, NetworkTableInstance, DoublePublisher

from wpimath.system.plant import DCMotor
from wpimath.units import amperes, radiansToRotations

from phoenix6.configs import (
    TalonFXConfiguration,
    OpenLoopRampsConfigs,
    CurrentLimitsConfigs,
    MotorOutputConfigs,
)
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from phoenix6.sim import TalonFXSimState
from phoenix6.status_signal import StatusSignal
from phoenix6.units import rotations_per_second


class OpenLoopWheel(Subsystem):
    ########## HARDWARE ##########
    _motor: TalonFX
    """
    The motor that spins the wheel.
    This is a krakenx60
    """

    _dutyCycleSetpoint: float = 0.0

    ########## CONFIGURATION ##########
    _canbus: str = "canivore"

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

    _currentLimit: amperes = 20

    ########## LOGGING ##########
    _nettable: NetworkTable

    _dutyCyclePublisher: DoublePublisher

    _velocityPublisher: DoublePublisher

    _currentPublisher: DoublePublisher

    _getDutyCycleSignal: StatusSignal[float]

    _getVelocitySignal: StatusSignal[rotations_per_second]

    _getCurrentSignal: StatusSignal[amperes]

    ########## SIMULATION ##########
    _motorSimState: TalonFXSimState

    def __init__(
        self,
        id: int,
        name: str,
        rampTime: float = 0.5,
        inverted: bool = False,
        shootingDutyCycle: float | None = None,
        idleDutyCycle: float | None = None,
    ) -> None:
        self._nettable = NetworkTableInstance.getDefault().getTable(f"000{name}")

        self._shootingDutyCyle = shootingDutyCycle or self._shootingDutyCyle
        self._idleDutyCycle = idleDutyCycle or self._idleDutyCycle
        self._motorInverted = inverted or self._motorInverted

        self._motor = TalonFX(id, self._canbus)

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
                MotorOutputConfigs()
                .with_inverted(
                    InvertedValue.CLOCKWISE_POSITIVE
                    if self._motorInverted
                    else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                )
                .with_neutral_mode(NeutralModeValue.COAST)
            )
        )

        self._currentPublisher = self._nettable.getDoubleTopic("Current amps").publish()
        self._velocityPublisher = self._nettable.getDoubleTopic(
            "Velocity rpm"
        ).publish()
        self._dutyCyclePublisher = self._nettable.getDoubleTopic(
            "DutyCycle %"
        ).publish()

        self._getCurrentSignal = self._motor.get_stator_current(False)
        self._getVelocitySignal = self._motor.get_velocity(False)
        self._getDutyCycleSignal = self._motor.get_duty_cycle(False)
        self._motorSimState = self._motor.sim_state

    def periodic(self) -> None:
        self._getCurrentSignal.refresh()
        self._getVelocitySignal.refresh()
        self._getDutyCycleSignal.refresh()

        self._currentPublisher.set(self._getCurrentSignal.value_as_double)
        self._velocityPublisher.set(self._getVelocitySignal.value_as_double)
        self._dutyCyclePublisher.set(self._getDutyCycleSignal.value_as_double)

        self._motor.set(self._dutyCycleSetpoint)

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
