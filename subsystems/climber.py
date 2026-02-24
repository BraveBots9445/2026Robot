from dataclasses import dataclass

from math import pi

from commands2 import Subsystem, Command

from ntcore import NetworkTableInstance, NetworkTable, StructPublisher

from wpilib import Servo, SmartDashboard, Mechanism2d, MechanismLigament2d, Color8Bit
from wpilib.simulation import ElevatorSim

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

from wpiutil.wpistruct import make_wpistruct

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

kINCHES_PER_FOOT = 12.0


@make_wpistruct
@dataclass
class ClimberData:
    # TODO: Refactor the motor raw stuff to be an external util
    positionIn: inches
    velocityInPerSec: inches
    positionSetpointIn: inches
    hookAngleSetpoint: Rotation2d
    hookAngleDegrees: degrees

    hookDutyCycle: float
    motorCurrent: amperes
    motorOutputPercent: float
    motorPositionRaw: rotation
    motorVelocityRaw: rotations_per_second


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

    _servoPort: int = 0

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

    _raisePidConfigs: Slot0Configs = (
        Slot0Configs()
        .with_k_p(0.7)
        .with_k_i(0.0)
        .with_k_d(0.0)
        .with_k_g(0.0)
        .with_k_s(0.0)
        # omit kv, ka
        .with_gravity_type(GravityTypeValue.ELEVATOR_STATIC)
    )

    _climbPidConfigs: Slot1Configs = (
        Slot1Configs()
        .with_k_p(1.00)
        .with_k_i(0.0)
        .with_k_d(0.0)
        .with_k_g(0.0)
        .with_k_s(0.0)
        # omit kv, ka
        .with_gravity_type(GravityTypeValue.ELEVATOR_STATIC)
    )

    ########## LOGGING ##########
    _nettable: NetworkTable

    _dataPublisher: StructPublisher
    """
    A publisher for the class data
    Publishes in ClimberData
    """

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
        self._nettable = NetworkTableInstance.getDefault().getTable("000Climber")

        self._motor = TalonFX(27, self._canbus)
        self._servo = Servo(self._servoPort)

        self._motorConfig = (
            TalonFXConfiguration()
            .with_slot0(self._raisePidConfigs)
            .with_slot1(self._climbPidConfigs)
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

        self._mechState = ClimberData(0, 0, 0, Rotation2d(), 0, 0, 0, 0, 0, 0)

        self._motor.configurator.apply(self._motorConfig)

        self._dataPublisher = self._nettable.getStructTopic(
            "ClimberData", ClimberData
        ).publish()

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

        self._elevatorSim = ElevatorSim(
            DCMotor.krakenX60(1),
            self._gearRatio,
            self._carriageMass,
            inchesToMeters(self._pulleyDiameter) / 2,
            -float("inf"),
            float("inf"),
            # inchesToMeters(self._minHeight - 5),
            # inchesToMeters(self._maxHeight + 5),
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

        self._positionSetpoint = self.getPositionInches()
        SmartDashboard.putData("Climber", self)
        SmartDashboard.putData("Climber Mech", mech)

    def periodic(self) -> None:
        self._currentSignal.refresh()
        self._dutyCycleSignal.refresh()
        self._rawPositionSignal.refresh()
        self._rawVelocitySignal.refresh()

        self._mechState.positionIn = self.getPositionInches()
        self._mechState.velocityInPerSec = self.getVelocityInchesPerSec()
        self._mechState.positionSetpointIn = self._positionSetpoint
        self._mechState.hookAngleSetpoint = self._hookAngleSetpoint
        self._mechState.hookAngleDegrees = self._servo.getAngle()

        slot = 0  # raise slot
        if self._mechState.positionSetpointIn <= self._mechState.positionIn:
            slot = 1  # lower slot

        setpointRaw = self._getInchesToRotations(self._positionSetpoint)
        self._motor.set_control(PositionVoltage(setpointRaw, slot=slot))
        self._servo.setAngle(self._hookAngleSetpoint.degrees())

        self._mechState.motorOutputPercent = self._dutyCycleSignal.value_as_double
        self._mechState.motorCurrent = self._currentSignal.value_as_double
        self._mechState.motorPositionRaw = self._rawPositionSignal.value_as_double
        self._mechState.motorVelocityRaw = self._rawVelocitySignal.value_as_double

        self._dataPublisher.set(self._mechState)

        self._elevatorMech.setLength(self._mechState.positionIn)
        self._elevatorSetpointMech.setLength(self._mechState.positionSetpointIn)
        self._hookMech.setAngle(self._mechState.hookAngleDegrees)
        self._setpointHookMech.setAngle(
            self._mechState.hookAngleSetpoint.degrees()
        )  # both hook angles should be the same

    def simulationPeriodic(self) -> None:
        if self._positionSetpoint > self._mechState.positionIn:
            # raise
            self._elevatorSim.setInputVoltage(self._motor.get() * 12)
            self._elevatorSim.update(0.02)
            self._climbElevatorSim.setState(
                self._elevatorSim.getPosition(), self._elevatorSim.getVelocity()
            )
            rotorVel = self._getFeetToRotations(self._elevatorSim.getVelocityFps())
        else:
            self._climbElevatorSim.setInputVoltage(self._motor.get() * 12)
            self._climbElevatorSim.update(0.02)
            self._elevatorSim.setState(
                self._climbElevatorSim.getPosition(),
                self._climbElevatorSim.getVelocity(),
            )
            rotorVel = self._getFeetToRotations(self._climbElevatorSim.getVelocityFps())

        self._motorSim.add_rotor_position(rotorVel * 0.02)
        self._motorSim.set_rotor_velocity(rotorVel)
        # print(rotorVel)

    def getPositionInches(self) -> inches:
        return self._getRotationsToInches(self._rawPositionSignal.value_as_double)

    def getVelocityInchesPerSec(self) -> inches:
        return self._getRotationsToInches(self._rawVelocitySignal.value_as_double)

    def atSetpoint(self) -> bool:
        return (
            abs(self._mechState.positionSetpointIn - self._mechState.positionIn)
            < self._tolerance
        )

    def setHeightSetpoint(self, height: inches) -> None:
        self._positionSetpoint = max(min(self._maxHeight, height), self._minHeight)

    def setHookSetpointDegrees(self, angle: degrees) -> None:
        self._hookAngleSetpoint = Rotation2d.fromDegrees(angle)

    def deployHook(self) -> None:
        self.setHookSetpointDegrees(0)

    def retractHook(self) -> None:
        self.setHookSetpointDegrees(90)

    def getHookDeployed(self) -> bool:
        return self._hookAngleSetpoint.degrees() < 85

    def _getRotationsToInches(self, rotations: rotation) -> inches:
        return rotations * self._pulleyDiameter * pi / self._gearRatio

    def _getInchesToRotations(self, inches: inches) -> rotation:
        return inches * self._gearRatio / (self._pulleyDiameter * pi)

    def _getFeetToRotations(self, feet: feet) -> rotation:
        return self._getInchesToRotations(feet * kINCHES_PER_FOOT)

    @property
    def minHeight(self) -> inches:
        return self._minHeight

    @property
    def maxHeight(self) -> inches:
        return self._maxHeight

    def getData(self) -> ClimberData:
        return self._mechState
