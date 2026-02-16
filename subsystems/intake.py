from commands2 import InstantCommand, Subsystem

from ntcore import (
    NetworkTable,
    NetworkTableInstance,
    DoublePublisher,
    StructPublisher,
    IntegerPublisher,
)

from wpilib import (
    reportError,
    Mechanism2d,
    MechanismLigament2d,
    Color8Bit,
    SmartDashboard,
    RobotBase,
)
from wpilib.simulation import SingleJointedArmSim

from wpimath.geometry import Rotation2d
from wpimath.units import (
    kilogram_square_meters,
    meters,
    inchesToMeters,
    radiansToRotations,
    degreesToRadians,
)
from wpimath.system.plant import DCMotor

from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
    CurrentLimitsConfigs,
    Slot0Configs,
    Slot1Configs,
    FeedbackConfigs,
    MagnetSensorConfigs,
    MotorOutputConfigs,
)
from phoenix6.controls import PositionDutyCycle
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.sim import TalonFXSimState, CANcoderSimState
from phoenix6.signals import (
    GravityTypeValue,
    NeutralModeValue,
    FeedbackSensorSourceValue,
    InvertedValue,
    SensorDirectionValue,
)
from phoenix6.status_signal import StatusSignal
from phoenix6.units import rotation, rotations_per_second, ampere


class Intake(Subsystem):
    """
    A (pivoting, for now) intake subsystem for intaking fuel.
    It has a pivot KrakenX60, a pivot CANcoder (WCP Throughbore), and a roller KrakenX60.
    The pivot is on closed loop position control, and the roller is on open loop duty cycle control.
    """

    ########## HARDWARE ##########
    _pivotMotor: TalonFX

    _pivotEncoder: CANcoder

    _rollerMotor: TalonFX

    ########## SETPOINTS ##########
    _pivotSetpoint: Rotation2d = Rotation2d.fromDegrees(90)

    _pivotClosedLoopSlot: int = 0
    """
    The slot currently being used for closed loop control of the pivot.
    0 if in the robot or moving to the outside, 1 if outside the robot
    """

    _rollerSetpoint: float = 0.0
    """
    Duty Cycle setpoint for the roller
    """

    ########## CONFIGURATION ##########
    _canbus: str = "canivore"

    _pivotGearRatio: float = 4 / 1
    """
    The gear ratio of the pivot mechanism.
    This is measured as (motor rotations) / (pivot rotations).
    """

    _pivotAbsoluteEncoderOffset: float = 0.0
    """
    The offset for the cancoder in rotations such that it reads 0 when the pivot is fully extended.
    """

    _pivotMotorDirection: InvertedValue = InvertedValue.CLOCKWISE_POSITIVE
    """
    The motor direction for the pivot such that a positive output pulls the intake in 
    """

    _rollerMotorDirection: InvertedValue = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    """
    The motor direction for the roller such that a positive output pulls fuel into the robot 
    """

    _pivotAbsoluteEncoderDirection: SensorDirectionValue = (
        SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
    )
    """
    The direction for the cancoder such that it increases when the intake is deployed 
    """

    _pivotMOI: kilogram_square_meters = (
        0.0161108325  # TODO: Recalculate this if the material != 6061
    )

    _pivotLength: meters = inchesToMeters(12.5)

    _pivotMotorConfig: TalonFXConfiguration

    _pivotEncoderConfig: CANcoderConfiguration

    _rollerMotorConfig: TalonFXConfiguration

    _pivotSlot0Config: Slot0Configs = (
        (
            Slot0Configs()
            .with_k_p(5.0)
            .with_k_i(0.0)
            .with_k_d(0.1)
            .with_k_g(0.0)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )
        if RobotBase.isSimulation()
        else (
            Slot0Configs()
            .with_k_p(1.0)
            .with_k_i(0.0)
            .with_k_d(0.1)
            .with_k_g(0.001)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )
    )
    """
    This is the PID configuration the pivot motor uses when moving between setpoints
    """

    _pivotSlot1Config: Slot1Configs = (
        Slot1Configs()
        .with_k_p(0.2)
        .with_k_i(0.0)
        .with_k_d(0.1)
        .with_k_g(0.1)
        .with_gravity_type(GravityTypeValue.ARM_COSINE)
    )
    """
    This is the PID configuration the pivot motor uses when outside of the robot
    """

    ########## LOGGING ##########
    _nettable: NetworkTable

    _pivotAngleMech: MechanismLigament2d

    _pivotAngleSetpointMech: MechanismLigament2d

    _pivotPositionPub: StructPublisher
    """
    Publishes in Rotation2d
    """

    _pivotPositionDegreesPub: DoublePublisher

    _pivotSetpointPub: StructPublisher
    """
    Publishes in Rotation2d
    """

    _pivotSetpointDegreesPub: DoublePublisher

    _pivotCurrentPub: DoublePublisher

    _pivotDutyCyclePub: DoublePublisher

    _pivotClosedLoopSlotPub: IntegerPublisher

    _pivotVelocityPub: DoublePublisher

    _rollerSetpointPub: DoublePublisher

    _rollerDutyCyclePub: DoublePublisher

    _rollerCurrentPub: DoublePublisher

    _rollerVelocityPub: DoublePublisher

    _pivotPositionSignal: StatusSignal[rotation]

    _pivotVelocitySignal: StatusSignal[rotations_per_second]

    _pivotCurrentSignal: StatusSignal[ampere]

    _pivotDutyCycleSignal: StatusSignal[float]

    _rollerDutyCycleSignal: StatusSignal[float]

    _rollerCurrentSignal: StatusSignal[ampere]

    _rollerVelocitySignal: StatusSignal[rotations_per_second]

    ########## SIMULATION ##########
    _pivotSimState: TalonFXSimState

    _encoderSimState: CANcoderSimState

    _rollerSimState: TalonFXSimState

    _pivotSim: SingleJointedArmSim

    def __init__(self) -> None:
        self._nettable = NetworkTableInstance.getDefault().getTable("000Intake")
        self._pivotMotor = TalonFX(40, self._canbus)
        self._pivotEncoder = CANcoder(41, self._canbus)
        self._rollerMotor = TalonFX(42, self._canbus)

        self._pivotMotorConfig = (
            TalonFXConfiguration()
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(30)
                .with_stator_current_limit_enable(True)
            )
            .with_slot0(self._pivotSlot0Config)
            .with_slot1(self._pivotSlot1Config)
            .with_feedback(
                FeedbackConfigs()
                .with_feedback_remote_sensor_id(self._pivotEncoder.device_id)
                .with_feedback_sensor_source(FeedbackSensorSourceValue.REMOTE_CANCODER)
                .with_rotor_to_sensor_ratio(self._pivotGearRatio)
            )
            .with_motor_output(
                MotorOutputConfigs()
                .with_inverted(self._pivotMotorDirection)
                .with_neutral_mode(NeutralModeValue.COAST)
            )
        )

        self._pivotEncoderConfig = CANcoderConfiguration().with_magnet_sensor(
            MagnetSensorConfigs().with_sensor_direction(
                self._pivotAbsoluteEncoderDirection
            )
        )

        self._rollerMotorConfig = (
            TalonFXConfiguration()
            .with_motor_output(
                MotorOutputConfigs()
                .with_inverted(self._rollerMotorDirection)
                .with_neutral_mode(NeutralModeValue.COAST)
            )
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(20)
                .with_stator_current_limit_enable(True)
            )
        )

        self._pivotMotor.configurator.apply(self._pivotMotorConfig)
        self._pivotEncoder.configurator.apply(self._pivotEncoderConfig)
        self._rollerMotor.configurator.apply(self._rollerMotorConfig)

        self._pivotCurrentPub = self._nettable.getDoubleTopic("Pivot/Current").publish()
        self._pivotPositionPub = self._nettable.getStructTopic(
            "Pivot/Position", Rotation2d
        ).publish()
        self._pivotSetpointPub = self._nettable.getStructTopic(
            "Pivot/Setpoint", Rotation2d
        ).publish()
        self._pivotPositionDegreesPub = self._nettable.getDoubleTopic(
            "Pivot/PositionDegrees"
        ).publish()
        self._pivotVelocityPub = self._nettable.getDoubleTopic(
            "Pivot/Velocity RPM"
        ).publish()
        self._pivotSetpointDegreesPub = self._nettable.getDoubleTopic(
            "Pivot/SetpointDegrees"
        ).publish()
        self._pivotDutyCyclePub = self._nettable.getDoubleTopic(
            "Pivot/DutyCycle"
        ).publish()
        self._pivotClosedLoopSlotPub = self._nettable.getIntegerTopic(
            "Pivot/ClosedLoopSlot"
        ).publish()

        self._rollerSetpointPub = self._nettable.getDoubleTopic(
            "Roller/Setpoint"
        ).publish()
        self._rollerDutyCyclePub = self._nettable.getDoubleTopic(
            "Roller/DutyCycle"
        ).publish()
        self._rollerCurrentPub = self._nettable.getDoubleTopic(
            "Roller/Current"
        ).publish()
        self._rollerVelocityPub = self._nettable.getDoubleTopic(
            "Roller/Velocity RPM"
        ).publish()

        self._pivotCurrentSignal = self._pivotMotor.get_stator_current(False)
        self._pivotDutyCycleSignal = self._pivotMotor.get_duty_cycle(False)
        self._pivotPositionSignal = self._pivotMotor.get_position(False)
        self._pivotVelocitySignal = self._pivotMotor.get_velocity(False)
        self._rollerCurrentSignal = self._rollerMotor.get_stator_current(False)
        self._rollerVelocitySignal = self._rollerMotor.get_velocity(False)
        self._rollerDutyCycleSignal = self._rollerMotor.get_duty_cycle(False)

        self._pivotSetpoint = self.getAngle()

        pivotMech = Mechanism2d(100, 100)
        self._pivotAngleMech = pivotMech.getRoot("Pivot Angle", 50, 50).appendLigament(
            "Angle", 50, self.getAngle().degrees()
        )
        self._pivotAngleSetpointMech = pivotMech.getRoot(
            "Pivot Setpoint", 50, 50
        ).appendLigament(
            "Setpoint", 50, self._pivotSetpoint.degrees(), color=Color8Bit(0, 0, 255)
        )

        self._pivotSim = SingleJointedArmSim(
            DCMotor.krakenX60(),
            self._pivotGearRatio,
            self._pivotMOI,
            self._pivotLength,
            -float("inf"),
            float("inf"),
            True,
            degreesToRadians(90),
        )

        self._pivotSimState = self._pivotMotor.sim_state
        self._encoderSimState = self._pivotEncoder.sim_state
        self._rollerSimState = self._rollerMotor.sim_state

        SmartDashboard.putData("Intake/Subsystem", self)
        SmartDashboard.putData("Intake/PivotMech", pivotMech)

    def periodic(self) -> None:
        self._pivotCurrentSignal.refresh()
        self._pivotDutyCycleSignal.refresh()
        self._pivotPositionSignal.refresh()
        self._rollerCurrentSignal.refresh()
        self._rollerVelocitySignal.refresh()
        self._rollerDutyCycleSignal.refresh()
        self._pivotVelocitySignal.refresh()

        pivotPosition = Rotation2d.fromRotations(
            self._pivotPositionSignal.value_as_double
        )
        self._pivotCurrentPub.set(self._pivotCurrentSignal.value_as_double)
        self._pivotDutyCyclePub.set(self._pivotDutyCycleSignal.value_as_double)
        self._pivotPositionPub.set(pivotPosition)
        self._pivotPositionDegreesPub.set(pivotPosition.degrees())
        self._pivotSetpointPub.set(self._pivotSetpoint)
        self._pivotSetpointDegreesPub.set(self._pivotSetpoint.degrees())
        self._pivotClosedLoopSlotPub.set(self._pivotClosedLoopSlot)
        self._pivotVelocityPub.set(self._pivotVelocitySignal.value_as_double)
        self._rollerCurrentPub.set(self._rollerCurrentSignal.value_as_double)
        self._rollerVelocityPub.set(self._rollerVelocitySignal.value_as_double)
        self._rollerDutyCyclePub.set(self._rollerDutyCycleSignal.value_as_double)

        self._pivotAngleMech.setAngle(pivotPosition.degrees())
        self._pivotAngleSetpointMech.setAngle(self._pivotSetpoint.degrees())

        self._pivotMotor.set_control(
            PositionDutyCycle(
                radiansToRotations(self._pivotSetpoint.radians()),
                slot=self._pivotClosedLoopSlot,
            )
        )
        self._rollerMotor.set(self._rollerSetpoint)

    def simulationPeriodic(self) -> None:
        self._pivotSim.setInputVoltage(self._pivotMotor.get() * 12)
        self._pivotSim.update(0.02)

        pivotVelocity = radiansToRotations(self._pivotSim.getVelocity())
        pivotRotorVelocity = pivotVelocity * self._pivotGearRatio
        self._pivotSimState.set_rotor_velocity(pivotRotorVelocity)
        self._pivotSimState.add_rotor_position(pivotRotorVelocity * 0.02)

        self._encoderSimState.set_velocity(pivotVelocity)
        self._encoderSimState.add_position(pivotVelocity * 0.02)

        rollerVelocity = radiansToRotations(
            self._rollerMotor.get() * DCMotor.krakenX60().freeSpeed
        )
        self._rollerSimState.set_rotor_velocity(rollerVelocity)
        self._rollerSimState.add_rotor_position(rollerVelocity * 0.02)

    def getAngle(self) -> Rotation2d:
        """
        Gets the current angle of the pivot of the intake.
        """
        return Rotation2d.fromRotations(self._pivotPositionSignal.value_as_double)

    def getDeployed(self) -> bool:
        """
        Gets whether the intake is currently deployed or not.
        """
        return self.getAngle().degrees() < 75

    def setPivotSetpoint(self, angle: Rotation2d) -> None:
        """
        Sets the pivot setpoint for the intake.

        :param angle: The angle to set the pivot to. 0 degrees is fully extended
        """
        self._pivotSetpoint = angle

    def setPivotSetpointDegrees(self, angleDegrees: float) -> None:
        """
        Sets the pivot setpoint for the intake in degrees.

        :param angleDegrees: The angle in degrees to set the pivot to. 0 degrees is fully extended
        """
        self.setPivotSetpoint(Rotation2d.fromDegrees(angleDegrees))

    def setRollerSetpoint(self, dutyCycle: float) -> None:
        """
        Sets the roller setpoint for the intake.

        :param dutyCycle: The duty cycle to set the roller to, from -1.0 to 1.0. positive is pulling fuel into the robot
        """
        self._rollerSetpoint = dutyCycle

    def setClosedLoopSlot(self, slot: int) -> None:
        """
        Sets the closed loop slot for the pivot PID control.

        :param slot: The slot to set, either 0 or 1
        """
        if slot not in (0, 1):
            reportError(f"Invalid slot {slot} for pivot closed loop control")
            return
        self._pivotClosedLoopSlot = slot

    def _tmpSetPivotSetpoinntCommand(self, angle: Rotation2d):

        return self.run(lambda: self.setPivotSetpoint(angle))
