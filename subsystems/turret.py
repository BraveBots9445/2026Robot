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
from wpimath.units import kilogram_square_meters, radiansToRotations, degrees
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

from phoenix6.hardware import CANcoder
from phoenix6.configs import CANcoderConfiguration
from phoenix6.status_signal import StatusSignal

from rev import (
    SparkMax,
    SparkMaxSim,
    RelativeEncoder,
    SparkRelativeEncoderSim,
    SparkBaseConfig,
    ResetMode,
    PersistMode,
    SparkClosedLoopController,
)


class Turret(Subsystem):
    """
    Subsystem to control the turret mechanism of the robot.
    This controls the aizmuth of the shooter.
    This subsystem is designed to not allow for wrap around between the -180 and 180 degree positions.

    It is controlled by a single TalonFX motor controller controlling a KrakenX60 (TODO: Is that true?).
    It is indexed by a CANCoder absolute encoder acting as a remote limit switch, seeing a manget on the turret carriage.
    """

    ########## HARDWARE ##########

    _motor: SparkMax
    """
    The motor controller for the turret controlling a neo 550
    """

    _motorClosedLoop: SparkClosedLoopController
    """
    The closed loop controller for the turret motor.
    """

    _encoder: RelativeEncoder
    """
    The motor attached encoder for the turret 
    """

    _cancoder: CANcoder
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
    _canbus: str = "canivore"
    """
    The CAN bus the the CANCoder is on. The turret is on the rio bus
    "canivore" for canivore, "" or "rio" for rio
    """

    _gearRatio: float = 10 / 1
    """
    The gear ratio of the turret mechanism.
    This is measured as motor rotations / turret rotations.
    """

    # motor PID gains
    _motorP: float = 0.5
    _motorI: float = 0.0
    _motorD: float = 0.05

    _canCoderConfig: CANcoderConfiguration
    """
    The current configuration for the turret CANCoder
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

    ########## SIMULATION ##########
    # _turretSim: SingleJointedArmSim
    # """
    # The simulation model for the turret.
    # A single jointed arm without gravity is a turret
    # """

    _motorSim: SparkMaxSim
    """
    The simulation object for the turret motor.
    """

    _encoderSim: SparkRelativeEncoderSim
    """
    The sim object for the turret motor encoder.
    """

    _motorSimModel: DCMotorSim
    """
    The simulation model for the turret motor.
    """

    _turretMOI: kilogram_square_meters = 0.01
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

        self._motor = SparkMax(23, SparkMax.MotorType.kBrushless)
        self._motorClosedLoop = self._motor.getClosedLoopController()
        self._encoder = self._motor.getEncoder()
        self._cancoder = CANcoder(24, self._canbus)

        motorConfig = SparkBaseConfig()
        motorConfig.smartCurrentLimit(20).secondaryCurrentLimit(25).setIdleMode(
            SparkBaseConfig.IdleMode.kCoast
        )
        motorConfig.closedLoop.pid(self._motorP, self._motorI, self._motorD)
        motorConfig.encoder.positionConversionFactor(
            1 / self._gearRatio
        ).velocityConversionFactor(1 / self._gearRatio)

        self._motor.configure(
            motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        )

        self._motorSim = SparkMaxSim(self._motor, DCMotor.NEO550())
        self._encoderSim = SparkRelativeEncoderSim(self._motor)

        self._canCoderMagnetStatusSignal = self._cancoder.get_fault_bad_magnet(False)

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
            DCMotor.NEO550(),
            self._gearRatio,
            self._turretMOI,
            0.0,
            -float("inf"),
            float("inf"),
            # degreesToRadians(-180),
            # degreesToRadians(180),
            False,
            0.0,
        )

        self._simMotor = SparkMaxSim(self._motor, DCMotor.NEO550())

        SmartDashboard.putData("Turret Mech", turretMech)
        SmartDashboard.putData("Turret", self)

    def periodic(self) -> None:
        self._canCoderMagnetStatusSignal.refresh()

        angle = self.getRotation()
        isMagnetDetected = self._canCoderMagnetStatusSignal.value
        self._rotationPub.set(angle)
        self._rotationSetpointPub.set(self._rotationSetpoint)
        self._motorCurrentPub.set(self._motor.getOutputCurrent())
        self._motorDutyCyclePub.set(self._motor.getAppliedOutput())
        self._canCoderMagnetStatusPub.set(isMagnetDetected)

        self._turretMech.setAngle(angle.degrees())
        self._turretSetpointMech.setAngle(self._rotationSetpoint.degrees())

        # reset the position of the motor when the cancoder magnet is detected
        # TODO: Do we need to mandate a low speed for this to happen?
        # TODO: Where is the cancoder/magnet physically located?
        if isMagnetDetected and RobotBase.isReal():
            self._encoder.setPosition(
                0.5
            )  # facing straight forward is 0.5 rotations (exactly in the middle of the -180 to 180)

        self._motorClosedLoop.setSetpoint(
            radiansToRotations(self._rotationSetpoint.radians() * self._gearRatio),
            SparkMax.ControlType.kPosition,
        )

    def simulationPeriodic(self) -> None:
        self._turretSim.setInputVoltage(self._motor.getAppliedOutput() * 12)

        self._turretSim.update(0.02)

        mechVel = self._turretSim.getVelocity() * self._gearRatio

        self._motorSim.iterate(mechVel, 12, 0.02)
        self._motorSim.setMotorCurrent(self._turretSim.getCurrentDraw())
        self._encoderSim.iterate(mechVel, 0.02)

    def setSetpoint(self, angle: Rotation2d) -> None:
        """
        Set the desired rotation setpoint of the turret.

        :param angle: The desired rotation setpoint of the turret. A Rotation2d.fromDegrees(0) is straight forward.
        :type angle: Rotation2d
        """
        self._rotationSetpoint = angle  # + Rotation2d.fromDegrees(180)

    def setSetpointDegrees(self, angle: degrees) -> None:
        """
        Set the desired rotation setpoint of the turret in degrees.

        :param angle: The desired rotation setpoint of the turret in degrees. 0 is straight forward, positive is counterclockwise, negative is clockwise.
        :type angle: degrees
        """
        self.setSetpoint(Rotation2d.fromDegrees(angle))

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
        return Rotation2d.fromRotations(self._encoder.getPosition() / self._gearRatio)

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
