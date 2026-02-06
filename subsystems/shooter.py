from math import pi

from commands2 import Subsystem, Command


from ntcore import NetworkTable, NetworkTableInstance, DoublePublisher, StructPublisher

from wpilib import (
    Mechanism2d,
    MechanismLigament2d,
    SmartDashboard,
    Color8Bit,
    RobotBase,
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
)
from wpimath.geometry import Rotation2d, Transform2d
from wpimath.system.plant import DCMotor, LinearSystemId


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
    _canBus: str = "canivore"
    """
    The CAN bus the turret motor and encoder are connected to.
    "canivore" for the CANivore CAN bus, "rio" or "" for the RoboRIO CAN bus.
    """

    _flywheelRadius: meters = inchesToMeters(2)
    """
    The radius in meters of the flywheel
    """

    _flywheelMOI: kilogram_square_meters = 0.05
    """
    The moment of inertia of all moving components of the flywheel system
    """

    _flywheelConfig: TalonFXConfiguration
    """
    Current configuration for the flywheel motor
    """

    _flywheelSlot0Configs: Slot0Configs = (
        Slot0Configs()
        .with_k_p(0.01)
        .with_k_i(0)
        .with_k_d(0.0)
        .with_k_s(0)
        .with_k_v(0)
        .with_k_a(0)
    )

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

    _hoodZeroOffset: float = 0.0
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

    _hoodMinAngle: Rotation2d = Rotation2d.fromDegrees(20)
    """
    The minimum angle of the hood. This is where the hood is fully retracted
    """

    _hoodMaxAngle: Rotation2d = Rotation2d.fromDegrees(40)
    """
    The max angle of the hood. This is where the hood is fully extended 
    """

    # hood PIDs
    _hoodP: float = 10.0
    _hoodI: float = 0.0
    _hoodD: float = 0.0

    _hoodkG: float = 0.2

    ########################## SETPOINTS ##########################

    _flywheelSetpoint: revolutions_per_minute = 0
    """
    The target speed for the flywheel in RPM
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

    _actualFlywheelSpeedPub: DoublePublisher
    """
    A publisher for the actual speed of the flywheel
    published in RPM
    """

    _desiredFlywheelSpeedPub: DoublePublisher
    """
    A publisher for the desired speed of the flywheel
    published in RPM
    """

    _actualHoodAnglePub: StructPublisher
    """
    A publisher for the actual hood angle
    published as a Rotation2d
    """

    _desiredHoodAnglePub: StructPublisher
    """
    A publisher for the desired hood angle
    published as a Rotation2d   
    """

    _motorCurrentPub: DoublePublisher
    """
    A publisher for the current draw of the flywheel motor in amps
    """

    _motorDutyCyclePub: DoublePublisher
    """
    A publisher for the duty cycle of the flywheel motor
    """

    _flywheelMech: MechanismLigament2d
    """
    A mechanism2d representation of the flywheel for visualization
    """

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

        self._flywheelMotor = TalonFX(20, self._canBus)
        self._hoodMotor = SparkMax(21, SparkMax.MotorType.kBrushless)
        self._hoodMotorClosedLoop = self._hoodMotor.getClosedLoopController()
        self._hoodEncoder = self._hoodMotor.getAbsoluteEncoder()

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
                .with_stator_current_limit(30)
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
        )
        hoodConfig.absoluteEncoder.zeroOffset(self._hoodZeroOffset).inverted(
            self._hoodAbsoluteEncoderInverted
        )
        hoodConfig.closedLoop.P(self._hoodP).I(self._hoodI).D(
            self._hoodD
        ).setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder).feedForward.kCos(
            self._hoodkG
        ).kCosRatio(
            self._hoodGearRatio
        )

        self._flywheelMotor.configurator.apply(self._flywheelConfig)
        self._hoodMotor.configure(
            hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        )

        self._getVelocitySignal = self._flywheelMotor.get_velocity(False)
        self._getFlywheelCurrentSignal = self._flywheelMotor.get_stator_current(False)
        self._getDutyCycleSignal = self._flywheelMotor.get_duty_cycle(False)

        self._flywheelMotorSimState = self._flywheelMotor.sim_state

        self._hoodMotorSim = SparkMaxSim(self._hoodMotor, DCMotor.NEO550())
        self._hoodEncoderSim = SparkAbsoluteEncoderSim(self._hoodMotor)

        self._actualFlywheelSpeedPub = self._nettable.getDoubleTopic(
            "Flywheel/ActualRPM"
        ).publish()
        self._desiredFlywheelSpeedPub = self._nettable.getDoubleTopic(
            "Flywheel/DesiredRPM"
        ).publish()

        self._motorCurrentPub = self._nettable.getDoubleTopic(
            "Flywheel/MotorCurrentAmps"
        ).publish()

        self._motorDutyCyclePub = self._nettable.getDoubleTopic(
            "Flywheel/MotorDutyCycle"
        ).publish()

        self._actualHoodAnglePub = self._nettable.getStructTopic(
            "Hood/ActualAngle", Rotation2d
        ).publish()
        self._desiredHoodAnglePub = self._nettable.getStructTopic(
            "Hood/DesiredAngle", Rotation2d
        ).publish()

        flywheelMech = Mechanism2d(100, 100)
        self._flywheelMech = flywheelMech.getRoot("flywheel", 50, 50).appendLigament(
            "flywheelPointer", 40, 0
        )
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

        SmartDashboard.putData("Shooter/Flywheel Mech", flywheelMech)
        SmartDashboard.putData("Shooter/Hood Mech", hoodMech)
        SmartDashboard.putData("Shooter/Subsystem", self)

    def periodic(self) -> None:
        # refresh signals
        self._getVelocitySignal.refresh()
        self._getFlywheelCurrentSignal.refresh()
        self._getDutyCycleSignal.refresh()

        # log data
        flywheelVelocity = self.getFlywheelVelocity()
        desiredFlywheelVelocity = self.getFlywheelSetpoint()
        hoodAngle = self.getHoodAngle()
        hoodAngleSetpoint = self.getHoodAngleSetpoint()
        self._actualFlywheelSpeedPub.set(flywheelVelocity)
        self._desiredFlywheelSpeedPub.set(desiredFlywheelVelocity)
        self._actualHoodAnglePub.set(hoodAngle)
        self._desiredHoodAnglePub.set(hoodAngleSetpoint)
        self._motorDutyCyclePub.set(self._getDutyCycleSignal.value_as_double)
        self._motorCurrentPub.set(self._getFlywheelCurrentSignal.value_as_double)

        # update mech2d
        self._flywheelMech.setAngle(
            self._flywheelMech.getAngle() + rotationsToDegrees(flywheelVelocity) * 0.02
        )
        self._hoodMech.setAngle(hoodAngle.degrees())
        self._hoodSetpointMech.setAngle(hoodAngleSetpoint.degrees())
        # set controls
        self._flywheelMotor.set_control(
            VelocityVoltage(
                self._flywheelSetpoint
                / self._flywheelConfig.feedback.sensor_to_mechanism_ratio
            )
        )

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
            self._flywheelSim.getAngularVelocity() / self._flywheelGearRatio
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
        # TODO: Do we want to constrain this to be positive or less than some maximum?
        self._flywheelSetpoint = setpoint

    def setHoodAngleSetpoint(self, setpoint: Rotation2d) -> None:
        """
        Sets the target hood angle for launching fuel
        :param setpoint: The target hood angle
        :type setpoint: Rotation2d
        """
        self._hoodAngleSetpoint = setpoint

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
        return (
            Rotation2d.fromRotations(self._hoodEncoder.getPosition())
            + self._hoodMinAngle
        )

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

        v0 = muzzleVelocity * hoodAngle.sin()
        det = (v0**2) - 2 * 9.81 * (launchHeight - impactHeight)
        if det < 0:
            return None

        timeToImpact = (v0 - det**0.5) / kGRAVITY_ACCELERATION

        return (
            Transform2d(
                muzzleVelocity * timeToImpact * hoodAngle.cos(),
                muzzleVelocity * timeToImpact * hoodAngle.sin(),
            ),
            timeToImpact,
        )
