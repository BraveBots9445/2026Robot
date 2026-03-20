from math import pi

from threading import Lock

from commands2 import Subsystem, Command, cmd

from ntcore import (
    NetworkTable,
    NetworkTableInstance,
)

from wpimath.geometry import Rotation2d
from wpimath.units import (
    kilogram_square_meters,
    radiansToRotations,
    degrees,
    degreesToRotations,
)
from wpimath import angleModulus
from wpimath.system.plant import DCMotor, LinearSystemId

from wpilib import (
    Mechanism2d,
    MechanismLigament2d,
    Color8Bit,
    RobotState,
    SmartDashboard,
    RobotBase,
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
    AbsoluteEncoder,
    FeedbackSensor,
)

from .BraveLogger import BraveLogger, TurretData


class Turret(Subsystem):
    """
    Subsystem to control the turret mechanism of the robot.
    This controls the aizmuth of the shooter.
    This subsystem is designed to not allow for wrap around between the -180 and 180 degree positions.

    It is controlled by a single Spark Max motor controller controlling a Neo 550
    It is indexed by a Rev Throughbore absolute encoder
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

    _absoluteEncoder: AbsoluteEncoder
    """
    The encoder for the turret. This is used for indexing.
    """

    ########## SETPOINTS ##########
    _rotationSetpoint: Rotation2d = Rotation2d()
    """
    The desired rotation setpoint for the turret.
    """

    _manualSetpointOffset: Rotation2d = Rotation2d()

    ########## CONFIGS ##########
    _canbus: str = ""
    """
    The CAN bus the the CANCoder is on. The turret is on the rio bus
    "canivore" for canivore, "" or "rio" for rio
    """

    _motorInverted: bool = True

    _gearRatio: float = 200 / 20
    """
    The gear ratio of the turret mechanism.
    This is measured as motor rotations / turret rotations.
    """

    _encoderInverted: bool = False

    _zeroOffset: float = 0.757604

    # motor PID gains
    _motorP: float = 5.5 if RobotBase.isReal() else 0.2
    _motorI: float = 0.0 if RobotBase.isReal() else 0.0
    _motorD: float = 0.0 if RobotBase.isReal() else 0.0

    _canCoderConfig: CANcoderConfiguration
    """
    The current configuration for the turret CANCoder
    """

    ########## LOGGING ##########
    _nettable: NetworkTable
    """
    The NetworkTable for logging turret data.
    """

    _data: TurretData

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

    _turretMOI: kilogram_square_meters = 0.030
    """
    The moment of inertia of the turret.
    This should come from CAD
    """

    _lock: Lock

    def __init__(self) -> None:
        self._lock = Lock()
        self._nettable = NetworkTableInstance.getDefault().getTable("000Turret")

        self._motor = SparkMax(24, SparkMax.MotorType.kBrushless)
        self._motorClosedLoop = self._motor.getClosedLoopController()
        absEncoder = self._motor.getAbsoluteEncoder()
        self._encoder = self._motor.getEncoder()

        motorConfig = SparkBaseConfig()
        motorConfig.smartCurrentLimit(55).setIdleMode(
            SparkBaseConfig.IdleMode.kCoast
        ).inverted(self._motorInverted)
        motorConfig.softLimit.forwardSoftLimit(
            degreesToRotations(175)
        ).forwardSoftLimitEnabled(True).reverseSoftLimit(
            degreesToRotations(-175)
        ).reverseSoftLimitEnabled(
            True
        )
        motorConfig.closedLoop.pid(self._motorP, self._motorI, self._motorD).maxOutput(
            1.0
        ).minOutput(-1.0).setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
        motorConfig.encoder.positionConversionFactor(
            1 / self._gearRatio
        ).velocityConversionFactor(1 / self._gearRatio)
        motorConfig.absoluteEncoder.inverted(self._encoderInverted).zeroOffset(
            self._zeroOffset
        ).zeroCentered(True).positionConversionFactor(1.0)

        self._motor.configure(
            motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        )

        self._encoder.setPosition(absEncoder.getPosition() / self._gearRatio)

        self._data = TurretData(0, 0, 0, 0)

        self._motorSim = SparkMaxSim(self._motor, DCMotor.NEO())
        self._encoderSim = SparkRelativeEncoderSim(self._motor)

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
            False,
            0.0,
        )

        self._simMotor = SparkMaxSim(self._motor, DCMotor.NEO550())

        self.setSetpoint(self.getRotation())

        self._lock = Lock()

        SmartDashboard.putData("Turret Mech", turretMech)
        SmartDashboard.putData("Turret", self)

    def periodic(self) -> None:
        angle = Rotation2d.fromRotations(self._encoder.getPosition())
        # with self._lock:
        self._data._rotationDegrees = angle.degrees()
        self._data._rotationSetpointDegrees = (
            self._rotationSetpoint.degrees() + self._manualSetpointOffset.degrees()
        )
        self._data._motorCurrent = self._motor.getOutputCurrent()
        self._data._motorDutyCycle = self._motor.getAppliedOutput()

        BraveLogger.pushSubsystemData(self._data)

        self._turretMech.setAngle(angle.degrees())
        self._turretSetpointMech.setAngle(self._rotationSetpoint.degrees())

        self._motorClosedLoop.setSetpoint(
            radiansToRotations(
                self._rotationSetpoint.radians() + self._manualSetpointOffset.radians()
            ),
            SparkMax.ControlType.kPosition,
        )

    def simulationPeriodic(self) -> None:
        self._turretSim.setInputVoltage(self._motor.getAppliedOutput() * 12)

        mechVel = self._turretSim.getVelocity() * self._gearRatio

        self._motorSim.iterate(mechVel, 12, 0.02)
        self._motorSim.setMotorCurrent(self._turretSim.getCurrentDraw())
        self._encoderSim.iterate(mechVel, 0.02)

        self._turretSim.update(0.02)

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
        # with self._lock:
        return Rotation2d.fromDegrees(self.getData()._rotationDegrees)

    def _rotation2dToRotations(self, angle: Rotation2d) -> float:
        return angleModulus(angle.radians()) / (2 * pi)

    def resetEncoder(self) -> None:
        """
        Resets the turret encoder to zero. This should only be used for testing, as in normal operation the encoder should be reset by the CANCoder when the magnet is detected.
        """
        return

    def _tmpResetCommand(self) -> Command:
        """
        A temporary command to reset the turret encoder to zero. This is for testing purposes only, as in normal operation the encoder should be reset by the CANCoder when the magnet is detected.

        :return A command that resets the turret encoder to zero when executed.
        :rtype: Command
        """
        return cmd.runOnce(self.resetEncoder).ignoringDisable(True)

    def atSetpoint(self) -> bool:
        return (
            abs(self._data._rotationDegrees - self._data._rotationSetpointDegrees) < 7
        )

    def getData(self) -> TurretData:
        """
        Get the current data of the turret.

        :return The current data of the turret.
        :rtype: TurretData
        """
        # with self._lock:
        return self._data

    def _tmpSetSetpointCommand(self, setpoint: Rotation2d) -> Command:
        return cmd.runOnce(lambda: self.setSetpoint(setpoint))

    def bumpManualOffset(self, bumpValueDegrees: degrees = 5) -> None:
        self._manualSetpointOffset += Rotation2d.fromDegrees(bumpValueDegrees)

    def dumpManualOffset(self, dumpValueDegrees: degrees = 5) -> None:
        self._manualSetpointOffset -= Rotation2d.fromDegrees(dumpValueDegrees)

    def bumpManualOffsetCommand(self, bumpValueDegrees: degrees = 5) -> Command:
        return cmd.runOnce(lambda: self.bumpManualOffset(bumpValueDegrees))

    def dumpManualOffsetCommand(self, dumpValueDegrees: degrees = 5) -> Command:
        return cmd.runOnce(lambda: self.dumpManualOffset(dumpValueDegrees))

    def resetManualOffset(self) -> None:
        self._manualSetpointOffset = Rotation2d()
