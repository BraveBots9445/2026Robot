from copy import deepcopy

from dataclasses import dataclass

from math import pi

from threading import Lock

from commands2 import Subsystem, Command

from ntcore import NetworkTable, NetworkTableInstance, DoublePublisher, StructPublisher
from ntcore.util import ntproperty

from wpilib import (
    Mechanism2d,
    MechanismLigament2d,
    SmartDashboard,
    Color8Bit,
)
from wpilib.simulation import FlywheelSim, SingleJointedArmSim

from wpimath.units import (
    revolutions_per_minute,
    seconds,
    meters,
    inchesToMeters,
    meters_per_second_squared,
    kilogram_square_meters,
    rotationsToDegrees,
    amperes,
    radiansToRotations,
    degrees,
)
from wpimath.geometry import Rotation2d, Transform2d
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.controller import BangBangController

from wpiutil.wpistruct import make_wpistruct


from phoenix6.hardware import TalonFX
from phoenix6.sim import TalonFXSimState
from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
    FeedbackConfigs,
    CurrentLimitsConfigs,
    MotorOutputConfigs,
)
from phoenix6.signals import NeutralModeValue
from phoenix6.status_signal import StatusSignal
from phoenix6.units import rotations_per_second
from phoenix6.controls import VelocityVoltage

from rev import (
    SparkMax,
    SparkBaseConfig,
    ResetMode,
    PersistMode,
    FeedbackSensor,
    AbsoluteEncoder,
    SparkMaxSim,
    SparkAbsoluteEncoderSim,
    SparkClosedLoopController,
    FeedForwardConfig,
)

from .BraveLogger import BraveLogger, ShooterData

kSECONDS_PER_MINUTE = 60
kGRAVITY_ACCELERATION: meters_per_second_squared = -9.81


class Shooter(Subsystem):
    """
    The shooter subsystem for the robot.
    This includes the flywheel and hooded shooter mechanisms.
    """

    ########################## HARDWARE ##########################
    _flywheelMotor: TalonFX
    """
    The motor that spins the flywheel
    """

    _hoodMotor: SparkMax
    """
    The SparkMax that controls the neo550 that controls the hood angle
    """

    _hoodMotorClosedLoop: SparkClosedLoopController
    """
    The closed loop controller for the hood motor to control it to the desired angle
    """

    _hoodEncoder: AbsoluteEncoder
    """
    The absolute encoder attached to the hood motor to measure the angle of the hood
    This is a Rev Throughbore encoder. 
    """

    ########################## CONFIGS ##########################
    _canBus: str = ""
    """
    The CAN bus the turret motor and encoder are connected to.
    "canivore" for the CANivore CAN bus, "rio" or "" for the RoboRIO CAN bus.
    """

    _useClosedLoopFlywheel = ntproperty("UseClosedLoopFlywheel", True)
    """
    If True, use closed loop velocity control on the TalonFX, otherwise, use Bang-Bang
    """

    _flywheelRadius: meters = inchesToMeters(2)
    """
    The radius in meters of the flywheel
    """

    _flywheelMOI: kilogram_square_meters = 0.01
    """
    The moment of inertia of all moving components of the flywheel system
    """

    _flywheelConfig: TalonFXConfiguration
    """
    Current configuration for the flywheel motor
    """

    _flywheelSlot0Configs: Slot0Configs = (
        Slot0Configs()
        .with_k_p(0.35)
        .with_k_i(0)
        .with_k_d(0.0)
        .with_k_s(0)
        .with_k_v(0.115)
        .with_k_a(0)
    )

    _flywheelBangBangController: BangBangController

    _flywheelGearRatio: float = 1 / 1
    """
    The ratio between rotations of the motor and rotations of the flywheel
    This is calculated as (motor rotations) / (flywheel rotations) 
    """

    _hoodGearRatio: float = 1 / 9
    """
    The gear ratio between the hood motor and the hood output. 
    This is calculated as (motor rotations) / (hood rotations)
    """

    _hoodZeroOffset: float = 0.9882542
    """
    The offset in rotations for the hood's absolute encoder to be considered the zero position of the hood (zero launch angle)
    """

    _hoodAbsoluteEncoderInverted: bool = False
    """
    Whether the absolute encoder is inverted relative to the motor
    """

    _hoodMOI: kilogram_square_meters = 0.0079
    """
    The moment of inertia of all moving components of the hood system
    This is about the axis of rotation of the hood
    Derive this from CAD. The real number is slightly higher than this (changing plastic, bolts)
    """

    _hoodArmLength: meters = inchesToMeters(9.5)

    _hoodMinAngle: Rotation2d = Rotation2d.fromDegrees(0)
    """
    The minimum angle of the hood. This is where the hood is fully retracted
    """

    _hoodMaxAngle: Rotation2d = Rotation2d.fromDegrees(50)
    """
    The max angle of the hood. This is where the hood is fully extended 
    """

    # hood PIDs
    _hoodP: float = 9.0
    _hoodI: float = 0.0
    _hoodD: float = 0.0

    _hoodkG: float = 0.2

    ########################## SETPOINTS ##########################

    _flywheelSetpoint: revolutions_per_minute = 0
    """
    The target speed for the flywheel in RPM
    """

    _flywheelFudgeFactor: float = 0.85
    """
    The number to multiply the flywheel setpoint by for changing system conditions
    """

    _hoodAngleSetpoint: Rotation2d = Rotation2d()
    """
    The desired hood angle that the fuel should launch from 
    This is measured from the horizontal 
    """

    ########################## LOGGING ##########################

    _nettable: NetworkTable
    """
    The networktable for the shooter to do logging with 
    """

    _data: ShooterData

    _lock: Lock

    _hoodMech: MechanismLigament2d
    """
    A mechanism2d representation of the hood for visualization
    """

    _hoodSetpointMech: MechanismLigament2d
    """
    A mechanism2d representation of the hood angle setpoint for visualization
    """

    _getVelocitySignal: StatusSignal[rotations_per_second]
    """
    The status signal cached to get the velocity of the flywheel in rotations per second
    """

    _getFlywheelCurrentSignal: StatusSignal[amperes]
    """
    The status signal cached to get the current draw of the flywheel motor in amps
    """

    _getDutyCycleSignal: StatusSignal[float]
    """
    The status signal cached to get the duty cycle of the motor
    """

    ########################## SIM ##########################

    _flywheelMotorSimState: TalonFXSimState
    """
    The sim state of the flywheel motor for simulation purposes
    """

    _hoodMotorSim: SparkMaxSim
    """
    The simulated SparkMax object for the hood
    """

    _hoodEncoderSim: SparkAbsoluteEncoderSim
    """
    The simulated SparkMax absolute encoder object for the hood
    """

    _flywheelSim: FlywheelSim
    """
    The simulated flywheel to improve fidelity of motor states
    """

    _hoodSim: SingleJointedArmSim
    """
    The simulated hood to improve fidelity of motor states
    """

    def __init__(self) -> None:
        self._nettable = NetworkTableInstance.getDefault().getTable("000Shooter")

        self._flywheelMotor = TalonFX(26, self._canBus)
        self._hoodMotor = SparkMax(25, SparkMax.MotorType.kBrushless)
        self._hoodMotorClosedLoop = self._hoodMotor.getClosedLoopController()
        self._hoodEncoder = self._hoodMotor.getAbsoluteEncoder()

        self._flywheelBangBangController = BangBangController()

        self._flywheelConfig = (
            TalonFXConfiguration()
            .with_slot0(self._flywheelSlot0Configs)
            .with_feedback(
                FeedbackConfigs().with_sensor_to_mechanism_ratio(
                    self._flywheelGearRatio
                )
            )
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(50)
                .with_stator_current_limit_enable(True)
            )
            .with_motor_output(
                MotorOutputConfigs().with_neutral_mode(NeutralModeValue.COAST)
            )
        )

        hoodConfig = (
            SparkBaseConfig()
            .smartCurrentLimit(20)
            .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
            .secondaryCurrentLimit(27)
            .inverted(True)
        )
        hoodConfig.absoluteEncoder.zeroOffset(self._hoodZeroOffset).inverted(
            self._hoodAbsoluteEncoderInverted
        )
        hoodConfig.closedLoop.P(self._hoodP).I(self._hoodI).D(
            self._hoodD
        ).positionWrappingEnabled(True).positionWrappingInputRange(
            0, 1
        ).allowedClosedLoopError(
            0.005
        ).setFeedbackSensor(
            FeedbackSensor.kAbsoluteEncoder
        ).feedForward.kCos(
            self._hoodkG
        )

        self._flywheelMotor.configurator.apply(self._flywheelConfig)
        self._hoodMotor.configure(
            hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        )

        self._getVelocitySignal = self._flywheelMotor.get_velocity(False)
        self._getFlywheelCurrentSignal = self._flywheelMotor.get_stator_current(False)
        self._getDutyCycleSignal = self._flywheelMotor.get_duty_cycle(False)

        # Reduce CAN bus traffic
        # self._flywheelMotor.optimize_bus_utilization()

        # Pre-allocate control request to reuse every cycle
        self._velocityVoltageRequest = VelocityVoltage(0)

        self._flywheelMotorSimState = self._flywheelMotor.sim_state

        self._hoodMotorSim = SparkMaxSim(self._hoodMotor, DCMotor.NEO550())
        self._hoodEncoderSim = SparkAbsoluteEncoderSim(self._hoodMotor)

        self._data = ShooterData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        hoodMech = Mechanism2d(100, 100)
        self._hoodMech = hoodMech.getRoot("hood", 50, 50).appendLigament(
            "hoodPointer", 40, 0
        )
        self._hoodSetpointMech = hoodMech.getRoot(
            "hoodSetpoint", 50, 50
        ).appendLigament("hoodSetpointPointer", 40, 0, color=Color8Bit(0, 0, 255))

        self._flywheelSim = FlywheelSim(
            LinearSystemId.flywheelSystem(
                DCMotor.krakenX60(1),
                self._flywheelMOI,
                self._flywheelGearRatio,
            ),
            DCMotor.krakenX60(1),
        )

        # TODO: This performs poorly - the simulated angle is far too high according to the sim, but the motor performs reasonably.
        self._hoodSim = SingleJointedArmSim(
            DCMotor.NEO550(),
            1 / self._hoodGearRatio,
            self._hoodMOI,
            self._hoodArmLength,
            -float("inf"),  # this is because of the angle problem
            float("inf"),
            # self._hoodMinAngle.radians(),
            # self._hoodMaxAngle.radians(),
            True,
            self._hoodMinAngle.radians(),
        )

        # SmartDashboard.putData("Shooter/Hood Mech", hoodMech)
        # SmartDashboard.putData("Shooter/Subsystem", self)

        self._lock = Lock()

        self._hoodAngleSetpoint = self.getHoodAngle()

    def periodic(self) -> None:
        StatusSignal.refresh_all(
            self._getVelocitySignal,
            self._getFlywheelCurrentSignal,
            self._getDutyCycleSignal,
        )

        # log data
        flywheelVelocity = self.getFlywheelVelocity()
        desiredFlywheelVelocity = self.getFlywheelSetpoint()
        hoodAngleSetpoint = self.getHoodAngleSetpoint()
        hoodAngle = (
            Rotation2d.fromRotations(self._hoodEncoder.getPosition())
            + self._hoodMinAngle
        )

        with self._lock:
            self._data.actualFlywheelSpeedRpm = flywheelVelocity
            self._data.desiredFlywheelSpeedRpm = desiredFlywheelVelocity
            # self._data.actualHoodAngle = hoodAngle
            # self._data.desiredHoodAngle = hoodAngleSetpoint
            self._data.actualHoodAngleDegrees = hoodAngle.degrees()
            self._data.desiredHoodAngleDegrees = hoodAngleSetpoint.degrees()
            self._data.motorDutyCycle = self._getDutyCycleSignal.value_as_double
            self._data.motorCurrent = self._getFlywheelCurrentSignal.value_as_double
            self._data.hoodMotorCurrent = self._hoodMotor.getOutputCurrent()

            BraveLogger.pushSubsystemData(deepcopy(self._data))

        # update mech2d
        # self._hoodMech.setAngle(hoodAngle.degrees())
        # self._hoodSetpointMech.setAngle(hoodAngleSetpoint.degrees())
        # set controls
        if abs(desiredFlywheelVelocity) < 50:
            self._flywheelMotor.set(0)
        # if self._useClosedLoopFlywheel:
        else:
            if self._useClosedLoopFlywheel:
                self._velocityVoltageRequest.velocity = (
                    self._flywheelSetpoint
                    / kSECONDS_PER_MINUTE
                    / self._flywheelConfig.feedback.sensor_to_mechanism_ratio
                    * self._flywheelFudgeFactor
                )
                self._flywheelMotor.set_control(self._velocityVoltageRequest)
            else:
                out = self._flywheelBangBangController.calculate(
                    abs(flywheelVelocity / kSECONDS_PER_MINUTE),
                    abs(
                        self._flywheelSetpoint
                        / kSECONDS_PER_MINUTE
                        * self._flywheelFudgeFactor
                    ),
                ) * (1 if self._flywheelSetpoint >= 0 else -1)
                self._flywheelMotor.set(out)

        self._hoodMotorClosedLoop.setSetpoint(
            (self._hoodAngleSetpoint - self._hoodMinAngle).degrees() / 360,
            SparkMax.ControlType.kPosition,
        )

    def simulationPeriodic(self) -> None:
        self._flywheelSim.setInput(
            [self._flywheelMotor.get_motor_voltage().value_as_double]
        )
        self._flywheelSim.update(0.02)

        self._flywheelMotorSimState.set_rotor_velocity(
            radiansToRotations(self._flywheelSim.getAngularVelocity())
            / self._flywheelGearRatio
        )

        self._hoodSim.setInputVoltage(
            self._hoodMotor.getAppliedOutput() * self._hoodMotor.getBusVoltage()
        )
        self._hoodSim.update(0.02)

        hoodVelocity = (
            radiansToRotations(self._hoodSim.getVelocity()) / self._hoodGearRatio
        )

        self._hoodMotorSim.iterate(
            hoodVelocity * self._hoodGearRatio,
            12,
            0.02,
        )
        self._hoodEncoderSim.iterate(hoodVelocity, 0.02)

    def setFlywheelSetpoint(self, setpoint: revolutions_per_minute) -> None:
        """
        Sets the target speed for the flywheel
        :param setpoint: The target speed in RPM
        :type setpoint: revolutions_per_minute
        """
        setpoint = max(min(setpoint, 6000), 0)
        self._flywheelSetpoint = setpoint

    def setHoodAngleSetpoint(self, setpoint: Rotation2d) -> None:
        """
        Sets the target hood angle for launching fuel
        :param setpoint: The target hood angle
        :type setpoint: Rotation2d
        """
        if setpoint.radians() < self._hoodMinAngle.radians():
            setpoint = self._hoodMinAngle
        elif setpoint.radians() > self._hoodMaxAngle.radians():
            setpoint = self._hoodMaxAngle
        self._hoodAngleSetpoint = setpoint

    def setHoodAngleSetpointDegrees(self, setpoint: degrees) -> None:
        """
        Sets the target hood angle in degrees for launching fuel

        :param setpoint: The target hood angle in degrees
        :type setpoint: degrees
        """
        self.setHoodAngleSetpoint(Rotation2d.fromDegrees(setpoint))

    def getFlywheelSetpoint(self) -> revolutions_per_minute:
        """
        Gets the current target speed for the flywheel
        :return: The target speed in RPM
        :rtype: revolutions_per_minute
        """
        return self._flywheelSetpoint

    def getHoodAngleSetpoint(self) -> Rotation2d:
        """
        Gets the current target hood angle
        :return: The target hood angle
        :rtype: Rotation2d
        """
        return self._hoodAngleSetpoint

    def getHoodAngle(self) -> Rotation2d:
        """
        Gets the current hood angle
        :return: The current hood angle
        :rtype: Rotation2d
        """
        with self._lock:
            return Rotation2d.fromDegrees(self._data.actualHoodAngleDegrees)

    def getHoodVelocity(self) -> rotations_per_second:
        """
        Gets the current angular velocity of the hood
        :return: The current hood angular velocity in rotations per second
        :rtype: rotations_per_second
        """
        return self._hoodEncoder.getVelocity() / 60

    def getFlywheelVelocity(self) -> revolutions_per_minute:
        """
        Gets the current speed of the flywheel
        :return: The current speed in RPM
        :rtype: revolutions_per_minute
        """
        # refreshes in periodic
        return (
            self._getVelocitySignal.value_as_double
            * kSECONDS_PER_MINUTE
            * self._flywheelGearRatio
        )

    def _tmpSetVelocityCommand(self, velocity: revolutions_per_minute) -> Command:
        """
        Temporary method to set the flywheel velocity for testing purposes
        :param velocity: The target speed in RPM
        :type velocity: revolutions_per_minute
        """
        return self.run(lambda: self.setFlywheelSetpoint(velocity))

    def _tmpSetHoodAngleCommand(self, angle: Rotation2d) -> Command:
        """
        Temporary method to set the hood angle for testing purposes
        :param angle: The target hood angle
        :type angle: Rotation2d
        """
        return self.run(lambda: self.setHoodAngleSetpoint(angle))

    def getEstimatedShotCharacteristics(
        self, launchHeight: meters, impactHeight: meters = 0
    ) -> tuple[Transform2d, seconds] | None:
        """
        Estimates the shot characteristics based on current flywheel speed and hood angle
        This should be used to ensure that the fuel will actually land in the target area when shooting, or otherwise, stop shooting temporarily

        :param launchHeight: The height from which the fuel is launched
        :type launchHeight: meters
        :param impactHeight: The height at which the fuel is intended to impact (default is 0)
        :type impactHeight: meters
        :return: A transform from the shooter to the impact point (at impactHeight), and time of flight to get there, or None if the height is unreachable in current configuration
        """

        hoodAngle = self.getHoodAngle()
        muzzleVelocity = self.getFlywheelVelocity() * 2 * pi * self._flywheelRadius

        v0 = muzzleVelocity * hoodAngle.cos()
        det = (v0**2) - 2 * 9.81 * (launchHeight - impactHeight)
        if det < 0:
            return None

        timeToImpact = (v0 - det**0.5) / kGRAVITY_ACCELERATION

        return (
            Transform2d(
                muzzleVelocity * timeToImpact * hoodAngle.cos(),
                muzzleVelocity * timeToImpact * hoodAngle.sin(),
                Rotation2d(),
            ),
            timeToImpact,
        )

    @property
    def minHoodAngle(self) -> Rotation2d:
        return self._hoodMinAngle

    @property
    def maxHoodAngle(self) -> Rotation2d:
        return self._hoodMaxAngle

    def getData(self) -> ShooterData:
        """
        Gets the current data for the shooter subsystem

        :return: The current data for the shooter subsystem
        :rtype: ShooterData
        """
        with self._lock:
            return self._data
