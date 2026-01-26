from math import pi

from commands2 import Subsystem, Command

from ntcore import (
    NetworkTable,
    NetworkTableInstance,
    StructPublisher,
    DoublePublisher,
    BooleanPublisher,
)

from wpimath.geometry import Rotation2d
from wpimath.units import amperes, kilogram_square_meters
from wpimath import angleModulus
from wpimath.system.plant import DCMotor, LinearSystemId

from wpilib import (
    Mechanism2d,
    MechanismLigament2d,
    SmartDashboard,
    RobotBase,
    Color8Bit,
)
from wpilib.simulation import DCMotorSim, SingleJointedArmSim

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
    Slot0Configs,
    FeedbackConfigs,
    CurrentLimitsConfigs,
    MotorOutputConfigs,
)
from phoenix6.status_signal import StatusSignal
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import PositionDutyCycle, PositionVoltage
from phoenix6.sim import TalonFXSimState


class Turret(Subsystem):
    """
    Subsystem to control the turret mechanism of the robot.
    This controls the aizmuth of the shooter.
    This subsystem is designed to not allow for wrap around between the -180 and 180 degree positions.

    It is controlled by a single TalonFX motor controller controlling a KrakenX60 (TODO: Is that true?).
    It is indexed by a CANCoder absolute encoder acting as a remote limit switch, seeing a manget on the turret carriage.
    """

    ########## HARDWARE ##########

    _motor: TalonFX
    """
    The motor controller for the turret.
    Controlling a Kraken X60 (TODO: Is that true?).
    """

    _canCoder: CANcoder
    """
    The encoder for the turret. This is used for indexing.
    When the magnet mounted on the turret passes the CANCoder, it's position is set to zero.
    """

    ########## SETPOINTS ##########
    _rotationSetpoint: Rotation2d = Rotation2d()
    """
    The desired rotation setpoint for the turret.
    """

    ########## CONFIGS ##########
    _canBus: str = "canivore"
    """
    The CAN bus the turret motor and encoder are connected to.
    "canivore" for the CANivore CAN bus, "rio" or "" for the RoboRIO CAN bus.
    """

    _gearRatio: float = 10 / 1
    """
    The gear ratio of the turret mechanism.
    This is measured as motor rotations / turret rotations.
    """

    _motorConfig: TalonFXConfiguration
    """
    The current configuration for the turret motor
    """

    _canCoderConfig: CANcoderConfiguration
    """
    The current configuration for the turret CANCoder
    """

    _closedLoopConfig: Slot0Configs = (
        Slot0Configs()
        .with_k_p(2.0)
        .with_k_i(0.0)
        .with_k_d(0.0)
        .with_k_s(0)
        .with_k_v(0)
        .with_k_a(0)
    )
    """
    The closed loop configuration for the turret motor.
    """

    ########## LOGGING ##########
    _nettable: NetworkTable
    """
    The NetworkTable for logging turret data.
    """

    _rotationPub: StructPublisher
    """
    Publisher for the turret rotation.
    Publishes in Rotation2d
    """

    _rotationSetpointPub: StructPublisher
    """
    Publisher for the turret rotation setpoint.
    Publishes in Rotation2d
    """

    _motorCurrentPub: DoublePublisher
    """
    Publisher for the turret motor current.
    Publishes in Amperes
    """

    _motorDutyCyclePub: DoublePublisher
    """
    Publisher for the turret motor duty cycle.
    Publishes in percentage (0.0 - 1.0)
    """

    _canCoderMagnetStatusPub: BooleanPublisher
    """
    Publisher for the CANCoder magnet status.
    Publishes True if the magnet is detected, False otherwise.
    """

    _turretMech: MechanismLigament2d
    """
    The ligament in the Mechanism2d for visualizing the turret angle.
    """

    _turretSetpointMech: MechanismLigament2d
    """
    The ligament in the Mechanism2d for visualizing the turret setpoint angle.
    """

    _canCoderMagnetStatusSignal: StatusSignal[bool]
    """
    The cached status signal to get the CANCoder magnet status.
    """

    _motorCurrentSignal: StatusSignal[amperes]
    """
    The cached status signal to get the motor current.
    """

    _motorDutyCycleSignal: StatusSignal[float]
    """
    The cached status signal to get the motor duty cycle.
    """

    _motorPositionSignal: StatusSignal[float]
    """
    The cached status signal to get the motor position.
    """

    ########## SIMULATION ##########
    # _turretSim: SingleJointedArmSim
    # """
    # The simulation model for the turret.
    # A single jointed arm without gravity is a turret
    # """

    _motorSimState: TalonFXSimState
    """
    The simulation state for the turret motor.
    """

    _motorSim: DCMotorSim
    """
    The simulation model for the turret motor.
    """

    _turretMOI: kilogram_square_meters = 0.06
    """
    The moment of inertia of the turret.
    This should come from CAD
    """

    # _turretRadius: meters = 0.3
    # """
    # The radius of the turret.
    # This should come from CAD.
    # This should be from the center of rotation to the furthest point on the turret.
    # """

    def __init__(self) -> None:
        self._nettable = NetworkTableInstance.getDefault().getTable("000Turret")

        self._motor = TalonFX(23, self._canBus)
        self._canCoder = CANcoder(24, self._canBus)

        self._motorConfig = (
            TalonFXConfiguration()
            # .with_feedback(
            #     FeedbackConfigs().with_sensor_to_mechanism_ratio(self._gearRatio)
            # )
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(40)
                .with_stator_current_limit_enable(True)
            )
            .with_motor_output(
                MotorOutputConfigs().with_neutral_mode(NeutralModeValue.COAST)
            )
            .with_slot0(self._closedLoopConfig)
        )

        self._canCoderMagnetStatusSignal = self._canCoder.get_fault_bad_magnet(False)
        self._motorCurrentSignal = self._motor.get_stator_current()
        self._motorDutyCycleSignal = self._motor.get_duty_cycle()
        self._motorPositionSignal = self._motor.get_position()

        self._rotationPub = self._nettable.getStructTopic(
            "Rotation", Rotation2d
        ).publish()
        self._rotationSetpointPub = self._nettable.getStructTopic(
            "RotationSetpoint", Rotation2d
        ).publish()

        self._motorCurrentPub = self._nettable.getDoubleTopic("MotorCurrent").publish()
        self._motorDutyCyclePub = self._nettable.getDoubleTopic(
            "MotorDutyCycle"
        ).publish()

        self._canCoderMagnetStatusPub = self._nettable.getBooleanTopic(
            "CanCoderMagnetStatus"
        ).publish()

        turretMech = Mechanism2d(100, 100)
        self._turretMech = turretMech.getRoot("Turret Angle", 50, 50).appendLigament(
            "Turret", 40, 0
        )
        self._turretSetpointMech = turretMech.getRoot(
            "Turret Setpoint Angle", 50, 50
        ).appendLigament("Turret Setpoint", 40, 0, color=Color8Bit(0, 0, 255))

        self._turretSim = SingleJointedArmSim(
            LinearSystemId.singleJointedArmSystem(
                DCMotor.krakenX60(1), self._turretMOI, self._gearRatio
            ),
            DCMotor.krakenX60(1),
            self._gearRatio,
            0.0,
            Rotation2d.fromDegrees(-180).radians(),
            Rotation2d.fromDegrees(180).radians(),
            False,
            0.0,
        )

        self._simMotor = self._motor.sim_state

        SmartDashboard.putData("Turret Mech", turretMech)
        SmartDashboard.putData("Turret", self)

    def periodic(self) -> None:
        self._motorCurrentSignal.refresh()
        self._motorDutyCycleSignal.refresh()
        self._motorCurrentSignal.refresh()
        self._motorPositionSignal.refresh()

        angle = self.getRotation()
        isMagnetDetected = self._canCoderMagnetStatusSignal.value
        self._rotationPub.set(angle)
        self._rotationSetpointPub.set(self._rotationSetpoint)
        self._motorCurrentPub.set(self._motorCurrentSignal.value_as_double)
        self._motorDutyCyclePub.set(self._motorDutyCycleSignal.value_as_double)
        self._canCoderMagnetStatusPub.set(isMagnetDetected)

        self._turretMech.setAngle(angle.degrees())
        self._turretSetpointMech.setAngle(self._rotationSetpoint.degrees())

        # reset the position of the motor when the cancoder magnet is detected
        # TODO: Do we need to mandate a low speed for this to happen?
        # TODO: Where is the cancoder/magnet physically located?
        if isMagnetDetected and RobotBase.isReal():
            self._motor.set_position(
                0.5
            )  # facing straight forward is 0.5 rotations (exactly in the middle of the -180 to 180)

        self._motor.set_control(
            PositionDutyCycle(
                # self._rotation2dToRotations(self._rotationSetpoint) / self._gearRatio
                self._rotationSetpoint.degrees()
                / 360
                * self._gearRatio
            )
        )

    def simulationPeriodic(self) -> None:
        self._turretSim.setInputVoltage(self._motor.get_motor_voltage().value_as_double)

        self._turretSim.update(0.02)

        mechPosition = self._turretSim.getAngle()
        mechVel = self._turretSim.getVelocity()

        rotorPosition = mechPosition * self._gearRatio / (2 * pi)
        rotorVelocity = mechVel * self._gearRatio / (2 * pi)

        self._simMotor.set_raw_rotor_position(rotorPosition)
        self._simMotor.set_rotor_velocity(rotorVelocity)

    def setSetpoint(self, angle: Rotation2d) -> None:
        """
        Set the desired rotation setpoint of the turret.

        :param angle: The desired rotation setpoint of the turret. A Rotation2d.fromDegrees(0) is straight forward.
        :type angle: Rotation2d
        """
        self._rotationSetpoint = angle  # + Rotation2d.fromDegrees(180)

    def getSetpoint(self) -> Rotation2d:
        """
        Get the current rotation setpoint of the turret.

        :return The current rotation setpoint of the turret.
        :rtype: Rotation2d
        """
        return self._rotationSetpoint

    def getRotation(self) -> Rotation2d:
        """
        Get the current rotation of the turret.

        :return The current rotation of the turret.
        :rtype: Rotation2d
        """
        return Rotation2d.fromDegrees(
            self._motorPositionSignal.value_as_double * 360 / self._gearRatio
        )

    def _rotation2dToRotations(self, angle: Rotation2d) -> float:
        return angleModulus(angle.radians()) / (2 * pi)

    def _tmpSetSetpointCommand(self, angle: Rotation2d) -> Command:
        """
        Temporary command to set the turret setpoint.
        Used for testing purposes.

        :param angle: The desired rotation setpoint of the turret.
        :type angle: Rotation2d
        :return: A command that sets the turret setpoint.
        :rtype: Command
        """

        return self.run(lambda: self.setSetpoint(angle))
