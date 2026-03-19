from enum import Enum

from math import pi

from threading import Lock

from commands2 import Subsystem

from ntcore import NetworkTableInstance, NetworkTable

from wpilib import Servo, Mechanism2d, MechanismLigament2d, Color8Bit
from wpilib.simulation import ElevatorSim

from .BraveLogger import BraveLogger

from wpimath.units import (
    inches,
    amperes,
    degrees,
    kilograms,
    lbsToKilograms,
    inchesToMeters,
    feet,
)
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor

from wpilib import SmartDashboard

from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
    Slot1Configs,
    CurrentLimitsConfigs,
    MotorOutputConfigs,
)
from phoenix6.controls import PositionVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import GravityTypeValue, InvertedValue, NeutralModeValue
from phoenix6.status_signal import StatusSignal
from phoenix6.sim import TalonFXSimState
from phoenix6.units import rotation, rotations_per_second

from .BraveLogger import ClimberData

kINCHES_PER_FOOT = 12.0


class Climber(Subsystem):
    """
    The subsystem to manage the active portion of the climber. (1 Kraken, 1 Servo for active hook)
    """

    ########## HARDWARE ##########
    _motor: TalonFX

    _servo: Servo

    ########## SETPOINTS ##########
    _positionSetpoint: inches
    _hookAngleSetpoint: Rotation2d = Rotation2d.fromDegrees(90)

    ########## CONFIGURATION ##########
    _canbus: str = "canivore1"

    _climbDutyCycle: float = 0.5

    _deployDutyCycle: float = -0.3

    _gearRatio: float = 16.0 / 1.0
    """
    The gear ratio between the motor shaft and pulley. This is measured in (motor rotations) / (pulley rotations). 
    """

    _pulleyDiameter: inches = 1.0
    """
    The diameter of the (big? TODO big or small) pulley in inches
    """

    _tolerance: inches = 0.5

    _minHeight: inches = 0.0
    _maxHeight: inches = 29.0

    _motorConfig: TalonFXConfiguration

    _inverted: InvertedValue = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    """
    The inversion of the motor such that positive is up. 
    """

    class State(Enum):
        IDLE = 0
        CLIMBING = 1
        DEPLOY = 2

    ########## LOGGING ##########
    _nettable: NetworkTable

    _mechState: ClimberData
    """
    The cached struct to write the current climber data to for publishing. This is used to avoid the overhead of creating a new struct every cycle.
    """

    _elevatorMech: MechanismLigament2d
    _elevatorSetpointMech: MechanismLigament2d
    _hookMech: MechanismLigament2d
    _setpointHookMech: MechanismLigament2d

    _rawPositionSignal: StatusSignal[rotation]
    _rawVelocitySignal: StatusSignal[rotations_per_second]
    _currentSignal: StatusSignal[amperes]
    _dutyCycleSignal: StatusSignal[float]

    ########## SIMULATION ##########
    _motorSim: TalonFXSimState

    # there are two elevator sims because one considers the mass of the robot and the other considers the mass of the carriage
    _elevatorSim: ElevatorSim  # just carriage
    _climbElevatorSim: ElevatorSim  # full robot

    _carriageMass: kilograms = lbsToKilograms(2.0)
    _robotMass: kilograms = lbsToKilograms(120.0)

    def __init__(self) -> None:
        self._motor = TalonFX(27, self._canbus)

        self._motorConfig = (
            TalonFXConfiguration()
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(80)
                .with_stator_current_limit_enable(True)
            )
            .with_motor_output(
                MotorOutputConfigs()
                .with_inverted(self._inverted)
                .with_neutral_mode(NeutralModeValue.BRAKE)
            )
        )

        self._mechState = ClimberData(0, 0, 0, 0, 0)

        self._motor.configurator.apply(self._motorConfig)

        mech = Mechanism2d(100, 100)
        root = mech.getRoot("Climber", 50, 50)
        betweenLen = 20
        self._elevatorMech = root.appendLigament("Elevator", 40, 90)
        betweenMech = self._elevatorMech.appendLigament(
            "Between", betweenLen, -90, color=Color8Bit(0, 255, 0)
        )
        self._hookMech = betweenMech.appendLigament(
            "Hook", 20, 90, color=Color8Bit(0, 255, 0)
        )

        self._elevatorSetpointMech = root.appendLigament(
            "Elevator Setpoint", 40, 90, color=Color8Bit(0, 0, 255)
        )
        setpointBetweenMech = self._elevatorSetpointMech.appendLigament(
            "Setpoint Between", betweenLen, -90, color=Color8Bit(0, 255, 255)
        )
        self._setpointHookMech = setpointBetweenMech.appendLigament(
            "Setpoint Hook", 20, 90, color=Color8Bit(0, 255, 255)
        )

        self._currentSignal = self._motor.get_stator_current(False)
        self._dutyCycleSignal = self._motor.get_duty_cycle(False)
        self._rawPositionSignal = self._motor.get_position(False)
        self._rawVelocitySignal = self._motor.get_velocity(False)

        BraveLogger.registerStatusSignal(
            [
                self._currentSignal,
                self._dutyCycleSignal,
                self._rawPositionSignal,
                self._rawVelocitySignal,
            ],
            bus=self._canbus,
        )

        self._positionVoltageRequest = PositionVoltage(0)

        self._elevatorSim = ElevatorSim(
            DCMotor.krakenX60(1),
            self._gearRatio,
            self._carriageMass,
            inchesToMeters(self._pulleyDiameter) / 2,
            -float("inf"),
            float("inf"),
            True,
            0,
        )

        self._climbElevatorSim = ElevatorSim(
            DCMotor.krakenX60(1),
            self._gearRatio,
            self._robotMass,
            inchesToMeters(self._pulleyDiameter) / 2,
            inchesToMeters(self._minHeight - 5),
            inchesToMeters(self._maxHeight + 5),
            True,
            0,
        )

        self._motorSim = TalonFXSimState(self._motor)

        SmartDashboard.putData("Climber", self)
        SmartDashboard.putData("Climber Mech", mech)

    def periodic(self) -> None:

        self._motor.set_control(self._positionVoltageRequest)

        self._mechState.motorOutputPercent = self._dutyCycleSignal.value_as_double
        self._mechState.motorCurrent = self._currentSignal.value_as_double
        self._mechState.motorPositionRaw = self._rawPositionSignal.value_as_double
        self._mechState.motorVelocityRaw = self._rawVelocitySignal.value_as_double

        if (
            abs(self._mechState.motorVelocityRaw) < 5
            and self._mechState.motorCurrent > 60
        ):
            self._mechState.state = Climber.State.IDLE.value

        if self._mechState.state == self.State.IDLE.value:
            self._motor.set(0)
        elif self._mechState.state == self.State.CLIMBING.value:
            self._motor.set(self._climbDutyCycle)
        elif self._mechState.state == self.State.DEPLOY.value:
            self._motor.set(self._deployDutyCycle)

        BraveLogger.pushSubsystemData(self._mechState)

    def simulationPeriodic(self) -> None:
        self._elevatorSim.setInputVoltage(
            self._motor.get_motor_voltage().value_as_double
        )

        rotorVel = self._elevatorSim.getVelocity()

        self._motorSim.add_rotor_position(rotorVel * 0.02)
        self._motorSim.set_rotor_velocity(rotorVel)

    def getData(self) -> ClimberData:
        return self._mechState

    def climb(self) -> None:
        self._mechState.state = Climber.State.CLIMBING.value

    def idleMode(self) -> None:
        self._mechState.state = Climber.State.IDLE.value

    def deploy(self) -> None:
        self._mechState.state = Climber.State.DEPLOY.value
