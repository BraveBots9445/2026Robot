from threading import Lock

from commands2 import Subsystem

from ntcore import (
    NetworkTable,
    NetworkTableInstance,
)

from wpilib import (
    reportError,
    Mechanism2d,
    MechanismLigament2d,
    Color8Bit,
    RobotBase,
    RobotState,
    SmartDashboard,
    Timer,
)
from wpilib.simulation import SingleJointedArmSim

from wpimath.geometry import Rotation2d
from wpimath.units import (
    degrees,
    kilogram_square_meters,
    meters,
    inchesToMeters,
    radiansToRotations,
    degreesToRadians,
    degreesToRotations,
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
    SoftwareLimitSwitchConfigs,
)
from phoenix6.controls import PositionDutyCycle, Follower
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.sim import TalonFXSimState, CANcoderSimState
from phoenix6.signals import (
    GravityTypeValue,
    NeutralModeValue,
    FeedbackSensorSourceValue,
    InvertedValue,
    SensorDirectionValue,
    MotorAlignmentValue,
)
from phoenix6.status_signal import StatusSignal
from phoenix6.units import rotation, rotations_per_second, ampere

from tools.BraveLogger import BraveLogger, IntakeData


class Intake(Subsystem):
    """
    A pivoting intake subsystem for intaking fuel.
    It has a pivot KrakenX60, a pivot CANcoder (WCP Throughbore), and a roller KrakenX60.
    The pivot is on closed loop position control, and the roller is on open loop duty cycle control.
    """

    ########## HARDWARE ##########
    _pivotMotor: TalonFX

    _pivotFollowerMotor: TalonFX

    _pivotEncoder: CANcoder

    _rollerMotor: TalonFX

    _pivotMotorCanId: int = 20

    _pivotFollowerMotorCanId: int = 21

    _pivotEncoderCanId: int = 20

    _rollerMotorCanId: int = 22

    ########## SETPOINTS ##########
    _pivotSetpoint: Rotation2d = Rotation2d.fromDegrees(0)

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
    _canbus: str = ""

    _pivotGearRatio: float = 27 / 1
    """
    The gear ratio of the pivot mechanism.
    This is measured as (motor rotations) / (pivot rotations).
    """

    _pivotAbsoluteEncoderOffset: float = 0.1667
    """
    The offset for the cancoder in rotations such that it reads 0 when the pivot is fully extended.
    """

    _pivotMotorDirection: InvertedValue = (
        InvertedValue.CLOCKWISE_POSITIVE
    )  # COUNTER_CLOCKWISE_POSITIVE
    """
    The motor direction for the pivot such that a positive output pulls the intake in 
    """

    _rollerMotorDirection: InvertedValue = InvertedValue.CLOCKWISE_POSITIVE
    """
    The motor direction for the roller such that a positive output pulls fuel into the robot 
    """

    _pivotAbsoluteEncoderDirection: SensorDirectionValue = (
        SensorDirectionValue.CLOCKWISE_POSITIVE
    )
    """
    The direction for the cancoder such that it increases when the intake is pulled in 
    """

    _pivotMotorConfig: TalonFXConfiguration

    _pivotFollowerMotorConfig: TalonFXConfiguration

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
            .with_k_d(0.0)
            .with_k_g(0.025)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )
    )
    """
    This is the PID configuration the pivot motor uses when moving between setpoints
    """

    _pivotSlot1Config: Slot1Configs = (
        Slot1Configs()
        .with_k_p(0.8)
        .with_k_i(0.0)
        .with_k_d(0.005)
        .with_k_g(0.015)
        .with_gravity_type(GravityTypeValue.ARM_COSINE)
    )
    """
    """

    ########## LOGGING ##########
    _nettable: NetworkTable

    _pivotAngleMech: MechanismLigament2d

    _pivotAngleSetpointMech: MechanismLigament2d

    _data: IntakeData

    _lock: Lock

    _pivotPositionSignal: StatusSignal[rotation]

    _pivotVelocitySignal: StatusSignal[rotations_per_second]

    _pivotCurrentSignal: StatusSignal[ampere]

    _pivotDutyCycleSignal: StatusSignal[float]

    _pivotFollowerCurrentSignal: StatusSignal[ampere]

    _pivotFollowerDutyCycleSignal: StatusSignal[float]

    _pivotFollowerVelocitySignal: StatusSignal[rotations_per_second]

    _rollerDutyCycleSignal: StatusSignal[float]

    _rollerCurrentSignal: StatusSignal[ampere]

    _rollerVelocitySignal: StatusSignal[rotations_per_second]

    ########## SIMULATION ##########
    _pivotSimState: TalonFXSimState

    _pivotFollowerSimState: TalonFXSimState

    _encoderSimState: CANcoderSimState

    _rollerSimState: TalonFXSimState

    _pivotSim: SingleJointedArmSim

    _pivotMOI: kilogram_square_meters = (
        0.0161108325  # TODO: Recalculate this if the material != 6061
    )

    _pivotLength: meters = inchesToMeters(12.5)

    def __init__(self) -> None:
        self._lock = Lock()
        self._nettable = NetworkTableInstance.getDefault().getTable("000Intake")
        self._pivotMotor = TalonFX(self._pivotMotorCanId, self._canbus)
        self._pivotFollowerMotor = TalonFX(self._pivotFollowerMotorCanId, self._canbus)
        self._pivotEncoder = CANcoder(self._pivotEncoderCanId, self._canbus)
        self._rollerMotor = TalonFX(self._rollerMotorCanId, self._canbus)

        self._pivotMotorConfig = (
            TalonFXConfiguration()
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(40)
                .with_stator_current_limit_enable(True)
            )
            .with_software_limit_switch(
                SoftwareLimitSwitchConfigs()
                .with_forward_soft_limit_enable(True)
                .with_reverse_soft_limit_enable(True)
                .with_forward_soft_limit_threshold(degreesToRotations(90))
                .with_reverse_soft_limit_threshold(degreesToRotations(-3.5))
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
            MagnetSensorConfigs()
            .with_sensor_direction(self._pivotAbsoluteEncoderDirection)
            .with_magnet_offset(self._pivotAbsoluteEncoderOffset)
        )

        self._pivotFollowerMotorConfig = (
            TalonFXConfiguration()
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(40)
                .with_stator_current_limit_enable(True)
            )
            .with_motor_output(
                MotorOutputConfigs()
                .with_inverted(self._pivotMotorDirection)
                .with_neutral_mode(NeutralModeValue.COAST)
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
                .with_stator_current_limit(40)
                .with_stator_current_limit_enable(True)
            )
        )

        self._pivotMotor.configurator.apply(self._pivotMotorConfig)
        self._pivotFollowerMotor.configurator.apply(self._pivotFollowerMotorConfig)
        self._pivotEncoder.configurator.apply(self._pivotEncoderConfig)
        self._rollerMotor.configurator.apply(self._rollerMotorConfig)

        self._pivotFollowerRequest = Follower(
            self._pivotMotor.device_id,
            MotorAlignmentValue.OPPOSED,
        )
        self._pivotFollowerMotor.set_control(self._pivotFollowerRequest)

        self._data = IntakeData(0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0)

        self._pivotCurrentSignal = self._pivotMotor.get_stator_current(False)
        self._pivotDutyCycleSignal = self._pivotMotor.get_duty_cycle(False)
        self._pivotPositionSignal = self._pivotMotor.get_position(False)
        self._pivotVelocitySignal = self._pivotMotor.get_velocity(False)
        self._pivotFollowerCurrentSignal = self._pivotFollowerMotor.get_stator_current(
            False
        )
        self._pivotFollowerDutyCycleSignal = self._pivotFollowerMotor.get_duty_cycle(
            False
        )
        self._pivotFollowerVelocitySignal = self._pivotFollowerMotor.get_velocity(False)
        self._rollerCurrentSignal = self._rollerMotor.get_stator_current(False)
        self._rollerVelocitySignal = self._rollerMotor.get_velocity(False)
        self._rollerDutyCycleSignal = self._rollerMotor.get_duty_cycle(False)

        BraveLogger.registerStatusSignal(
            [
                self._pivotCurrentSignal,
                self._pivotDutyCycleSignal,
                self._pivotPositionSignal,
                self._pivotVelocitySignal,
                self._pivotFollowerCurrentSignal,
                self._pivotFollowerDutyCycleSignal,
                self._pivotFollowerVelocitySignal,
                self._rollerCurrentSignal,
                self._rollerVelocitySignal,
                self._rollerDutyCycleSignal,
            ]
        )

        self._positionDutyCycleRequest = PositionDutyCycle(0)

        self._pivotSimState = self._pivotMotor.sim_state
        self._pivotFollowerSimState = self._pivotFollowerMotor.sim_state
        self._encoderSimState = self._pivotEncoder.sim_state
        self._rollerSimState = self._rollerMotor.sim_state

        self._pivotSim = SingleJointedArmSim(
            DCMotor.krakenX60(2),
            self._pivotGearRatio,
            self._pivotMOI,
            self._pivotLength,
            -float("inf"),
            float("inf"),
            True,
            degreesToRadians(90),
        )
        self._encoderSimState.set_raw_position(
            radiansToRotations(-self._pivotSim.getAngle())
            + self._pivotAbsoluteEncoderOffset
        )

        self.setPivotSetpoint(
            Rotation2d.fromRotations(
                self._pivotEncoder.get_absolute_position().value_as_double
            )
        )

        pivotMech = Mechanism2d(100, 100)
        self._pivotAngleMech = pivotMech.getRoot("Pivot Angle", 50, 50).appendLigament(
            "Angle", 50, self.getAngle().degrees()
        )
        self._pivotAngleSetpointMech = pivotMech.getRoot(
            "Pivot Setpoint", 50, 50
        ).appendLigament(
            "Setpoint", 50, self._pivotSetpoint.degrees(), color=Color8Bit(0, 0, 255)
        )
        self._pivotAngleSimMech = pivotMech.getRoot("Pivot Sim", 50, 50).appendLigament(
            "Sim", 50, self._pivotSim.getAngleDegrees(), color=Color8Bit(0, 255, 0)
        )

        SmartDashboard.putData("Intake/Subsystem", self)
        SmartDashboard.putData("Intake/PivotMech", pivotMech)

        self._data.pivotPositionDegrees = Rotation2d.fromRotations(
            self._pivotPositionSignal.value_as_double
        ).degrees()

        self._stallTimer = Timer()
        self._reverseTimer = Timer()

    def periodic(self) -> None:
        if RobotState.isDisabled():
            self.setPivotSetpoint(self.getAngle())

        pivotPosition = Rotation2d.fromRotations(
            self._pivotPositionSignal.value_as_double
        )
        self._data.pivotPositionDegrees = pivotPosition.degrees()
        self._data.pivotSetpointDegrees = self._pivotSetpoint.degrees()
        self._data.pivotCurrent = self._pivotCurrentSignal.value_as_double
        self._data.pivotDutyCycle = self._pivotDutyCycleSignal.value_as_double
        self._data.pivotClosedLoopSlot = self._pivotClosedLoopSlot
        self._data.pivotVelocity = self._pivotVelocitySignal.value_as_double
        self._data.rollerSetpoint = self._rollerSetpoint
        self._data.rollerDutyCycle = self._rollerDutyCycleSignal.value_as_double
        current = self._rollerCurrentSignal.value_as_double
        velocity = self._rollerVelocitySignal.value_as_double
        self._data.rollerCurrent = current
        self._data.rollerVelocity = velocity

        self._rollerMotor.set(self._rollerSetpoint)
        BraveLogger.pushSubsystemData(self._data)

        self._pivotAngleMech.setAngle(pivotPosition.degrees())
        self._pivotAngleSetpointMech.setAngle(self._pivotSetpoint.degrees())

        self._positionDutyCycleRequest.position = radiansToRotations(
            self._pivotSetpoint.radians()
        )
        self._pivotMotor.set_control(self._positionDutyCycleRequest)
        self._pivotFollowerMotor.set_control(self._pivotFollowerRequest)

    def simulationPeriodic(self) -> None:
        self._pivotSim.setInputVoltage(self._pivotMotor.get() * 12)
        self._pivotAngleSimMech.setAngle(self._pivotSim.getAngleDegrees())

        pivotVelocity = radiansToRotations(self._pivotSim.getVelocity())
        # pivotRotorVelocity = pivotVelocity * self._pivotGearRatio
        # self._pivotSimState.set_rotor_velocity(pivotRotorVelocity)
        # self._pivotSimState.add_rotor_position(pivotRotorVelocity * 0.02)

        # self._pivotFollowerSimState.set_rotor_velocity(-pivotRotorVelocity)
        # self._pivotFollowerSimState.add_rotor_position(-pivotRotorVelocity * 0.02)

        self._encoderSimState.set_velocity(-pivotVelocity)  # / self._pivotGearRatio)
        self._encoderSimState.add_position(
            -pivotVelocity * 0.02
        )  # / self._pivotGearRatio * 0.02)

        rollerVelocity = radiansToRotations(
            self._rollerMotor.get() * DCMotor.krakenX60().freeSpeed
        )
        self._rollerSimState.set_rotor_velocity(rollerVelocity)
        self._rollerSimState.add_rotor_position(rollerVelocity * 0.02)

        self._pivotSim.update(0.02)

    def getAngle(self) -> Rotation2d:
        """
        Gets the current angle of the pivot of the intake.
        """
        return Rotation2d.fromDegrees(self.getData().pivotPositionDegrees)

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

    def atSetpoint(self, tolerance: degrees = 5) -> bool:
        """
        Checks if the pivot is at its setpoint within a certain tolerance.

        :param tolerance: The tolerance to check in degrees
        :return: Whether the pivot is at its setpoint within the given tolerance
        :rtype: bool
        """
        return (
            abs(self.getAngle().degrees() - self._pivotSetpoint.degrees()) < tolerance
        )

    def getData(self) -> IntakeData:
        """
        Gets the current data for the intake subsystem.

        :return: The current data for the intake subsystem.
        :rtype: IntakeData
        """
        # with self._lock:
        return self._data
