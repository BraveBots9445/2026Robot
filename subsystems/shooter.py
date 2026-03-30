from math import pi

from threading import Lock

from commands2 import Subsystem, Command
from commands2 import cmd

from ntcore import NetworkTable, NetworkTableInstance
from ntcore.util import ntproperty

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
    amperes,
    radiansToRotations,
    degrees,
    degreesToRadians,
)
from wpimath.geometry import Rotation2d, Transform2d
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.controller import BangBangController, ArmFeedforward

from phoenix6.hardware import TalonFX
from phoenix6.sim import TalonFXSimState
from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
    FeedbackConfigs,
    CurrentLimitsConfigs,
    MotorOutputConfigs,
    HardwareLimitSwitchConfigs,
)
from phoenix6.signals import (
    NeutralModeValue,
    ForwardLimitSourceValue,
    ReverseLimitSourceValue,
    ForwardLimitTypeValue,
    ReverseLimitTypeValue,
)
from phoenix6.status_signal import StatusSignal
from phoenix6.units import rotations_per_second
from phoenix6.controls import VelocityVoltage, PositionVoltage


from tools.BraveLogger import BraveLogger, ShooterData

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

    _hoodMotor: TalonFX

    ########################## CONFIGS ##########################
    _canBus: str = "canivore1"
    """
    The CAN bus the flywheel and hood motors are connected to.
    "canivore" for the CANivore CAN bus, "rio" or "" for the RoboRIO CAN bus.
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

    _hoodFeedForward: ArmFeedforward

    _flywheelGearRatio: float = 1 / 1
    """
    The ratio between rotations of the motor and rotations of the flywheel
    This is calculated as (motor rotations) / (flywheel rotations) 
    """

    _hoodConfig: TalonFXConfiguration

    # TODO: Validate
    _hoodGearRatio: float = 170 / (17 * 5)
    """
    The gear ratio between the hood motor and the hood output. 
    This is calculated as (motor rotations) / (hood rotations)
    """

    _hoodMOI: kilogram_square_meters = 0.0079
    """
    The moment of inertia of all moving components of the hood system
    This is about the axis of rotation of the hood
    Derive this from CAD. The real number is slightly higher than this (changing plastic, bolts)
    """

    _hoodArmLength: meters = inchesToMeters(9.5)

    _hoodMinAngle: Rotation2d = Rotation2d.fromDegrees(-5)
    """
    The minimum angle of the hood. This is where the hood is fully retracted
    """

    _hoodMaxAngle: Rotation2d = Rotation2d.fromDegrees(63)
    """
    The max angle of the hood. This is where the hood is fully extended 
    """

    # hood PIDs
    _hoodSlot0Configs: Slot0Configs = (
        Slot0Configs().with_k_p(1.0).with_k_i(0).with_k_d(0)
    )

    ########################## SETPOINTS ##########################

    _flywheelSetpoint: revolutions_per_minute = 0
    """
    The target speed for the flywheel in RPM
    """

    # _flywheelFudgeFactor = ntproperty("flywheelFudgeFactor", 0.975)
    _flywheelFudgeFactor = ntproperty("flywheelFudgeFactor", 1.0)
    """
    The number to multiply the flywheel setpoint by for changing system conditions
    """

    _hoodFudgeFactor = ntproperty("hoodFudgeFactor", 0.0)
    """
    The number to add the hood angle setpoint by for changing system conditions, in degrees
    """

    _hoodAngleSetpoint: Rotation2d = Rotation2d()
    """
    The desired hood angle that the fuel should launch from 
    This is measured from the horizontal 
    """

    _forceNotReady: bool = False
    """When true, isReady() will always return false."""

    ########################## LOGGING ##########################
    _data: ShooterData

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
    The status signal cached to get the duty cycle of the flywheel motor
    """

    _getHoodPositionSignal: StatusSignal[float]
    """
    The status signal cached to get the position of the hood motor in rotations
    """

    _getHoodVelocitySignal: StatusSignal[rotations_per_second]
    """
    The status signal cached to get the velocity of the hood motor in rotations per second
    """

    _getHoodCurrentSignal: StatusSignal[amperes]
    """
    The status signal cached to get the current draw of the hood motor in amps
    """

    _getHoodDutyCycleSignal: StatusSignal[float]
    """
    The status signal cached to get the duty cycle of the hood motor
    """

    ########################## SIM ##########################

    _flywheelMotorSimState: TalonFXSimState
    """
    The sim state of the flywheel motor for simulation purposes
    """

    _hoodMotorSimState: TalonFXSimState

    _flywheelSim: FlywheelSim
    """
    The simulated flywheel to improve fidelity of motor states
    """

    _hoodSim: SingleJointedArmSim
    """
    The simulated hood to improve fidelity of motor states
    """

    def __init__(self) -> None:
        self._flywheelMotor = TalonFX(26, self._canBus)
        self._hoodMotor = TalonFX(27, self._canBus)

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

        self._hoodConfig = (
            TalonFXConfiguration()
            .with_slot0(self._hoodSlot0Configs)
            .with_feedback(
                FeedbackConfigs().with_sensor_to_mechanism_ratio(self._hoodGearRatio)
            )
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(30)
                .with_stator_current_limit_enable(RobotBase.isReal())
            )
            # TODO: Hardware limit switches based on CANDi
            # requires validating that the wiring is correct, and what the acutal angles are
            # .with_hardware_limit_switch(
            #     HardwareLimitSwitchConfigs()
            #     .with_forward_limit_enable(True)
            #     .with_forward_limit_source(ForwardLimitSourceValue.REMOTE_CANDI_S1)
            #     .with_forward_limit_remote_sensor_id(27)
            #     .with_forward_limit_autoset_position_enable(True)
            #     .with_forward_limit_autoset_position_value(
            #         self._hoodMaxAngle.degrees() / 360 * self._hoodGearRatio
            #     )
            #     .with_reverse_limit_enable(True)
            #     .with_reverse_limit_source(ReverseLimitSourceValue.REMOTE_CANDI_S2)
            #     .with_reverse_limit_remote_sensor_id(27)
            #     .with_reverse_limit_autoset_position_enable(True)
            #     .with_reverse_limit_autoset_position_value(
            #         self._hoodMinAngle.degrees() / 360 * self._hoodGearRatio
            #     )
            # )
        )

        self._flywheelMotor.configurator.apply(self._flywheelConfig)
        self._hoodMotor.configurator.apply(self._hoodConfig)

        self._getVelocitySignal = self._flywheelMotor.get_velocity(False)
        self._getFlywheelCurrentSignal = self._flywheelMotor.get_stator_current(False)
        self._getDutyCycleSignal = self._flywheelMotor.get_duty_cycle(False)

        self._getHoodPositionSignal = self._hoodMotor.get_position(False)
        self._getHoodVelocitySignal = self._hoodMotor.get_velocity(False)
        self._getHoodCurrentSignal = self._hoodMotor.get_stator_current(False)
        self._getHoodDutyCycleSignal = self._hoodMotor.get_duty_cycle(False)

        BraveLogger.registerStatusSignal(
            [
                self._getVelocitySignal,
                self._getFlywheelCurrentSignal,
                self._getDutyCycleSignal,
                self._getHoodPositionSignal,
                self._getHoodVelocitySignal,
                self._getHoodCurrentSignal,
                self._getHoodDutyCycleSignal,
            ],
            self._canBus,
        )

        self._velocityVoltageRequest = VelocityVoltage(0)
        self._hoodPositionVoltageRequest = PositionVoltage(0)

        self._hoodFeedForward = ArmFeedforward(0, 0.5, 0, 0)

        self._flywheelMotorSimState = self._flywheelMotor.sim_state

        self._hoodMotorSimState = self._hoodMotor.sim_state

        self._data = ShooterData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

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

        self._hoodSim = SingleJointedArmSim(
            DCMotor.krakenX60(),
            self._hoodGearRatio,
            self._hoodMOI,
            self._hoodArmLength,
            -float("inf"),
            float("inf"),
            False,
            self._hoodMinAngle.radians(),
        )

        SmartDashboard.putData("Shooter/Hood Mech", hoodMech)
        SmartDashboard.putData("Shooter/Subsystem", self)

        self._data.actualHoodAngleDegrees = Rotation2d.fromRotations(
            self._getHoodPositionSignal.value_as_double
        ).degrees()
        self._data.desiredHoodAngleDegrees = self._data.actualHoodAngleDegrees

        self._hoodAngleSetpoint = self.getHoodAngle()

    def periodic(self) -> None:
        # log data
        flywheelVelocity = (
            self._getVelocitySignal.value_as_double
            * kSECONDS_PER_MINUTE
            * self._flywheelGearRatio
        )
        desiredFlywheelVelocity = self.getFlywheelSetpoint()
        hoodAngleSetpoint = self.getHoodAngleSetpoint()
        hoodAngle = Rotation2d.fromRotations(
            self._getHoodPositionSignal.value_as_double
        )
        hoodVelocity = self._getHoodVelocitySignal.value_as_double

        self._data.actualFlywheelSpeedRpm = flywheelVelocity
        self._data.desiredFlywheelSpeedRpm = desiredFlywheelVelocity
        self._data.actualHoodAngleDegrees = hoodAngle.degrees()
        self._data.desiredHoodAngleDegrees = hoodAngleSetpoint.degrees()
        self._data.flywheelMotorDutyCycle = self._getDutyCycleSignal.value_as_double
        self._data.flywheelMotorCurrent = self._getFlywheelCurrentSignal.value_as_double
        self._data.hoodMotorCurrent = self._getHoodCurrentSignal.value_as_double
        self._data.hoodMotorDutyCycle = self._getHoodDutyCycleSignal.value_as_double
        self._data.hoodMotorVelocity = hoodVelocity
        self._data.hoodMotorPosition = self._getHoodPositionSignal.value_as_double

        BraveLogger.pushSubsystemData(self._data)

        # update mech2d
        self._hoodMech.setAngle(hoodAngle.degrees())
        self._hoodSetpointMech.setAngle(hoodAngleSetpoint.degrees())
        # set controls
        if abs(desiredFlywheelVelocity) < 50:
            self._flywheelMotor.set(0)
        else:
            self._velocityVoltageRequest.velocity = (
                self._flywheelSetpoint
                / kSECONDS_PER_MINUTE
                / self._flywheelConfig.feedback.sensor_to_mechanism_ratio
            )
            self._flywheelMotor.set_control(self._velocityVoltageRequest)

        self._hoodMotor.set_control(
            self._hoodPositionVoltageRequest.with_position(
                hoodAngleSetpoint.degrees() / 360 * self._hoodGearRatio
            )
        )

    def simulationPeriodic(self) -> None:

        self._flywheelSim.setInputVoltage(
            self._flywheelMotor.get_motor_voltage().value_as_double
        )

        self._flywheelSim.update(0.02)

        self._flywheelMotorSimState.set_rotor_velocity(
            # radiansToRotations(self._flywheelSim.getAngularVelocity())
            # / self._flywheelGearRatio
            self._flywheelSetpoint
            / kSECONDS_PER_MINUTE
        )

        self._hoodSim.setInputVoltage(
            self._hoodMotor.get_motor_voltage().value_as_double
        )
        hoodVelocity = (
            radiansToRotations(self._hoodSim.getVelocity()) * self._hoodGearRatio
        )
        hoodVelocity = DCMotor.krakenX60().freeSpeed * self._hoodMotor.get()

        self._hoodMotorSimState.set_rotor_velocity(hoodVelocity)
        self._hoodMotorSimState.add_rotor_position(hoodVelocity * 0.02)

        self._hoodSim.update(0.02)

    def setFlywheelSetpoint(self, setpoint: revolutions_per_minute) -> None:
        """
        Sets the target speed for the flywheel
        :param setpoint: The target speed in RPM
        :type setpoint: revolutions_per_minute
        """
        setpoint = max(min(setpoint, 6000), 0) * self._flywheelFudgeFactor
        self._flywheelSetpoint = setpoint

    def setHoodAngleSetpoint(
        self, setpoint: Rotation2d, ignoreManual: bool = False
    ) -> None:
        """
        Sets the target hood angle for launching fuel
        :param setpoint: The target hood angle
        :type setpoint: Rotation2d
        """
        if not ignoreManual:
            setpoint += Rotation2d.fromDegrees(self._hoodFudgeFactor)
        if setpoint.radians() < self._hoodMinAngle.radians():
            setpoint = self._hoodMinAngle
        elif setpoint.radians() > self._hoodMaxAngle.radians():
            setpoint = self._hoodMaxAngle
        self._hoodAngleSetpoint = setpoint

    def setHoodAngleSetpointDegrees(
        self, setpoint: degrees, ignoreManual: bool = False
    ) -> None:
        """
        Sets the target hood angle in degrees for launching fuel

        :param setpoint: The target hood angle in degrees
        :type setpoint: degrees
        """
        self.setHoodAngleSetpoint(
            Rotation2d.fromDegrees(setpoint), ignoreManual=ignoreManual
        )

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
        return Rotation2d(self.getData().actualHoodAngleDegrees)

    def getHoodVelocity(self) -> rotations_per_second:
        """
        Gets the current angular velocity of the hood
        :return: The current hood angular velocity in rotations per second
        :rtype: rotations_per_second
        """
        return self._data.hoodMotorVelocity

    def getFlywheelVelocity(self) -> revolutions_per_minute:
        """
        Gets the current speed of the flywheel
        :return: The current speed in RPM
        :rtype: revolutions_per_minute
        """
        # refreshes in periodic
        return self._data.actualFlywheelSpeedRpm

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

    def atFlywheelSetpoint(self) -> bool:
        return (
            abs(self._data.actualFlywheelSpeedRpm - self._data.desiredFlywheelSpeedRpm)
            < 50
        )

    def atHoodSetpoint(self) -> bool:
        return (
            abs(self._data.actualHoodAngleDegrees - self._data.desiredHoodAngleDegrees)
            < 3
        )

    def isReady(self) -> bool:
        """
        Returns whether the shooter is ready to fire.

        Ready requires:
        - flywheel target speed is non-zero,
        - flywheel is at target speed,
        - hood is at target position.
        """
        if self._forceNotReady:
            return False

        flywheelTarget = abs(self.getFlywheelSetpoint())
        return (
            flywheelTarget >= 50 and self.atFlywheelSetpoint() and self.atHoodSetpoint()
        )

    def setForceNotReady(self, enabled: bool) -> None:
        """
        Force isReady() to return false when enabled.

        :param enabled: True to force not-ready, False to allow normal readiness behavior.
        :type enabled: bool
        """
        self._forceNotReady = enabled

    def getData(self) -> ShooterData:
        """
        Gets the current data for the shooter subsystem

        :return: The current data for the shooter subsystem
        :rtype: ShooterData
        """
        return self._data

    def bumpFlywheelFudge(self, bumpVal: float = 0.025) -> None:
        self._flywheelFudgeFactor += bumpVal

    def dumpFlywheelFudge(self, dumpVal: float = 0.025) -> None:
        self._flywheelFudgeFactor -= dumpVal

    def bumpFlywheelFudgeCommand(self, bumpVal: float = 0.025) -> Command:
        return cmd.runOnce(lambda: self.bumpFlywheelFudge(bumpVal))

    def dumpFlywheelFudgeCommand(self, dumpVal: float = 0.025) -> Command:
        return cmd.runOnce(lambda: self.dumpFlywheelFudge(dumpVal))

    def bumpHoodFudge(self, bumpVal: float = 1.0) -> None:
        self._hoodFudgeFactor += bumpVal

    def dumpHoodFudge(self, dumpVal: float = 1.0) -> None:
        self._hoodFudgeFactor -= dumpVal

    def bumpHoodFudgeCommand(self, bumpVal: float = 1.0) -> Command:
        return cmd.runOnce(lambda: self.bumpHoodFudge(bumpVal))

    def dumpHoodFudgeCommand(self, dumpVal: float = 1.0) -> Command:
        return cmd.runOnce(lambda: self.dumpHoodFudge(dumpVal))
