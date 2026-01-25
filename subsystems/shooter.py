from math import pi

from commands2 import Subsystem, Command


from ntcore import NetworkTable, NetworkTableInstance, DoublePublisher, StructPublisher

from wpilib import Servo, Mechanism2d, MechanismLigament2d, SmartDashboard
from wpilib.simulation import FlywheelSim

from wpimath.units import (
    revolutions_per_minute,
    seconds,
    meters,
    inchesToMeters,
    meters_per_second_squared,
    kilogram_square_meters,
    rotationsToDegrees,
    amperes,
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
from phoenix6.controls import VelocityVoltage, Follower

kSECONDS_PER_MINUTE = 60
kGRAVITY_ACCELERATION: meters_per_second_squared = -9.81


class Shooter(Subsystem):
    """
    The shooter subsystem for the robot.
    This includes the flywheel and hooded shooter mechanisms.
    """

    ########################## MOTORS ##########################
    _flywheelMasterMotor: TalonFX
    """
    The leader motor that spins the flywheel
    """

    _flywheelFollowerMotor: TalonFX
    """
    The follower motor that spins the flywheel
    """

    # TODO: These servos will be controlled on a Rev Servo Hub probably.
    # Therefore, the object will not be a wpilib.Servo, but rather a Rev Servo Hub servo object.
    # Leaving as wpilib.Servo for now for simplicity and since Rev hub is no confirmed.
    _hoodAngleLeft: Servo
    """
    The linear actuator servo that adjusts the hood angle on the left side
    Found here: https://www.studica.co/linear-rc-actuator-140mm-200n
    """
    _hoodAngleRight: Servo
    """
    The linear actuator servo that adjusts the hood angle on the right side
    Found here: https://www.studica.co/linear-rc-actuator-140mm-200n
    """

    ########################## CONFIGS ##########################
    _canBus: str = "canivore"
    """
    The CAN bus the turret motor and encoder are connected to.
    "canivore" for the CANivore CAN bus, "rio" or "" for the RoboRIO CAN bus.
    """

    _flywheelRadius: meters = inchesToMeters(4)
    """
    The radius in meters of the flywheel
    """

    _flywheelMOI: kilogram_square_meters = 0.05
    """
    The moment of inertia of all moving components of the flywheel system
    """

    _flywheelMasterConfig: TalonFXConfiguration
    """
    Current configuration for the master flywheel motor
    """

    _flywheelFollowerConfig: TalonFXConfiguration
    """
    Current configuration for the follower flywheel motor
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

    _gearRatio: float = 1 / 4
    """
    The ratio between rotations of the motor and rotations of the flywheel
    This is calculated as (flywheel rotations) / (motor rotations)
    """

    ########################## SETPOINTS ##########################

    _flyWheelSetpoint: revolutions_per_minute = 0
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

    _masterMotorCurrentPub: DoublePublisher
    """
    A publisher for the current draw of the master flywheel motor in amps
    """

    _followerMotorCurrentPub: DoublePublisher
    """
    A publisher for the current draw of the follower flywheel motor in amps
    """

    _masterMotorDutyCyclePub: DoublePublisher
    """
    A publisher for the duty cycle of the master flywheel motor
    """

    _followerMotorDutyCyclePub: DoublePublisher
    """
    A publisher for the duty cycle of the follower flywheel motor
    """

    _flywheelMech: MechanismLigament2d
    """
    A mechanism2d representation of the flywheel for visualization
    """

    _hoodMech: MechanismLigament2d
    """
    A mechanism2d representation of the hood for visualization
    """

    _getVelocitySignal: StatusSignal[rotations_per_second]
    """
    The status signal cached to get the velocity of the flywheel in rotations per second
    """

    _getMasterCurrentSignal: StatusSignal[amperes]
    """
    The status signal cached to get the current draw of the master motor in amps
    """

    _getFollowerCurrentSignal: StatusSignal[amperes]
    """
    The status signal cached to get the current draw of the follower motor in amps
    """

    _getMasterDutyCycleSignal: StatusSignal[float]
    """
    The status signal cached to get the duty cycle of the master motor
    """

    _getFollowerDutyCycleSignal: StatusSignal[float]
    """
    The status signal cached to get the duty cycle of the follower motor
    """

    ########################## SIM ##########################

    _flywheelMasterMotorSimState: TalonFXSimState
    """
    The sim state of the master flywheel motor for simulation purposes
    """

    _flywheelFollowerMotorSimState: TalonFXSimState
    """
    The sim state of the follower flywheel motor for simulation purposes
    """

    _flywheelSim: FlywheelSim
    """
    The simulated flywheel to improve fidelity of motor states
    """

    def __init__(self) -> None:
        self._nettable = NetworkTableInstance.getDefault().getTable("000Shooter")

        self._flywheelMasterMotor = TalonFX(20, self._canBus)
        self._flywheelFollowerMotor = TalonFX(21, self._canBus)

        currentLimits = (
            CurrentLimitsConfigs()
            .with_stator_current_limit(70)
            .with_stator_current_limit_enable(True)
        )
        self._flywheelMasterConfig = (
            TalonFXConfiguration()
            .with_slot0(self._flywheelSlot0Configs)
            .with_feedback(
                FeedbackConfigs().with_sensor_to_mechanism_ratio(
                    self._gearRatio * kSECONDS_PER_MINUTE
                )
            )
            .with_current_limits(currentLimits)
            .with_motor_output(
                MotorOutputConfigs().with_neutral_mode(NeutralModeValue.COAST)
            )
        )

        self._flywheelFollowerConfig = (
            TalonFXConfiguration()
            .with_current_limits(currentLimits)
            .with_motor_output(
                MotorOutputConfigs().with_neutral_mode(NeutralModeValue.COAST)
            )
        )  # this will follow, so no closed loop configuration

        self._flywheelMasterMotor.configurator.apply(self._flywheelMasterConfig)
        self._flywheelFollowerMotor.configurator.apply(self._flywheelFollowerConfig)

        self._getVelocitySignal = self._flywheelMasterMotor.get_velocity(False)
        self._getMasterCurrentSignal = self._flywheelMasterMotor.get_stator_current(
            False
        )
        self._getFollowerCurrentSignal = self._flywheelFollowerMotor.get_stator_current(
            False
        )
        self._getMasterDutyCycleSignal = self._flywheelMasterMotor.get_duty_cycle(False)
        self._getFollowerDutyCycleSignal = self._flywheelFollowerMotor.get_duty_cycle(
            False
        )

        self._flywheelMasterMotorSimState = self._flywheelMasterMotor.sim_state
        self._flywheelFollowerMotorSimState = self._flywheelFollowerMotor.sim_state

        self._hoodAngleLeft = Servo(0)
        self._hoodAngleRight = Servo(1)

        self._actualFlywheelSpeedPub = self._nettable.getDoubleTopic(
            "Flywheel/ActualRPM"
        ).publish()
        self._desiredFlywheelSpeedPub = self._nettable.getDoubleTopic(
            "Flywheel/DesiredRPM"
        ).publish()

        self._masterMotorCurrentPub = self._nettable.getDoubleTopic(
            "Flywheel/MasterMotorCurrentAmps"
        ).publish()
        self._followerMotorCurrentPub = self._nettable.getDoubleTopic(
            "Flywheel/FollowerMotorCurrentAmps"
        ).publish()

        self._masterMotorDutyCyclePub = self._nettable.getDoubleTopic(
            "Flywheel/MasterMotorDutyCycle"
        ).publish()
        self._followerMotorDutyCyclePub = self._nettable.getDoubleTopic(
            "Flywheel/FollowerMotorDutyCycle"
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

        self._flywheelSim = FlywheelSim(
            LinearSystemId.flywheelSystem(
                DCMotor.krakenX60(2),
                self._flywheelMOI,
                self._gearRatio,
            ),
            DCMotor.krakenX60(2),
        )

        SmartDashboard.putData("Shooter Flywheel Mech", flywheelMech)
        SmartDashboard.putData("Shooter Hood Mech", hoodMech)
        SmartDashboard.putData("Shooter", self)

    def periodic(self) -> None:
        # refresh signals
        self._getVelocitySignal.refresh()
        self._getMasterCurrentSignal.refresh()
        self._getFollowerCurrentSignal.refresh()
        self._getMasterDutyCycleSignal.refresh()
        self._getFollowerDutyCycleSignal.refresh()

        # log data
        flywheelVelocity = self.getFlywheelVelocity()
        desiredFlywheelVelocity = self.getFlywheelSetpoint()
        hoodAngle = self.getHoodAngle()
        hoodAngleSetpoint = self.getHoodAngleSetpoint()
        self._actualFlywheelSpeedPub.set(flywheelVelocity)
        self._desiredFlywheelSpeedPub.set(desiredFlywheelVelocity)
        self._actualHoodAnglePub.set(hoodAngle)
        self._desiredHoodAnglePub.set(hoodAngleSetpoint)
        self._masterMotorDutyCyclePub.set(
            self._getMasterDutyCycleSignal.value_as_double
        )
        self._followerMotorDutyCyclePub.set(
            self._getFollowerDutyCycleSignal.value_as_double
        )
        self._masterMotorCurrentPub.set(self._getMasterCurrentSignal.value_as_double)
        self._followerMotorCurrentPub.set(
            self._getFollowerCurrentSignal.value_as_double
        )

        # update mech2d
        self._flywheelMech.setAngle(
            self._flywheelMech.getAngle() + rotationsToDegrees(flywheelVelocity) * 0.02
        )
        self._hoodMech.setAngle(hoodAngle.degrees())

        # set controls
        self._flywheelMasterMotor.set_control(
            VelocityVoltage(
                self._flyWheelSetpoint
                / self._flywheelMasterConfig.feedback.sensor_to_mechanism_ratio
            )
        )

        class tmpBool:  # this is to make set_control not fail when calling .value on the bool passed in
            value: bool = True

        self._flywheelFollowerMotor.set_control(
            Follower(self._flywheelMasterMotor.device_id, tmpBool())  # type: ignore
        )

    def simulationPeriodic(self) -> None:
        self._flywheelSim.setInput(
            [self._flywheelMasterMotor.get_motor_voltage().value_as_double * 2]
        )
        print(self._flywheelMasterMotor.get_motor_voltage().value_as_double * 2)
        self._flywheelSim.update(0.02)
        self._flywheelSim.getAngularVelocity()

        self._flywheelMasterMotorSimState.set_rotor_velocity(
            self._flywheelSim.getAngularVelocity() / self._gearRatio
        )
        self._flywheelFollowerMotorSimState.set_rotor_velocity(
            self._flywheelSim.getAngularVelocity() / self._gearRatio
        )

    def setFlywheelSetpoint(self, setpoint: revolutions_per_minute) -> None:
        """
        Sets the target speed for the flywheel
        :param setpoint: The target speed in RPM
        :type setpoint: revolutions_per_minute
        """
        # TODO: Do we want to constrain this to be positive or less than some maximum?
        self._flyWheelSetpoint = setpoint

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
        return self._flyWheelSetpoint

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
            self.getHoodAngleSetpoint()
        )  # TODO: not sure how to sim this yet, so just return the setpoint

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
            * self._gearRatio
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
