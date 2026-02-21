from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6 import configs, signals
from rev import SparkMax, SparkMaxConfig, ResetMode, PersistMode, SparkMaxSim
from wpimath.controller import PIDController
from wpilib import RobotBase
from wpilib.simulation import FlywheelSim, SingleJointedArmSim
from wpimath.system.plant import DCMotor, LinearSystemId
from ntcore import NetworkTableInstance
import math
import numpy as np


class Shooter(Subsystem):
    """
    Shooter subsystem with single flywheel and adjustable hood.
    Flywheel uses Kraken X60 motor with built-in encoder for velocity feedback.
    Hood uses Neo550 motor with ThroughBore encoder for position feedback.
    """

    def __init__(
        self,
        flywheel_motor_id: int = 25,
        hood_motor_id: int = 26,
    ):
        """
        Initialize the Shooter subsystem.

        :param flywheel_motor_id: CAN ID for the flywheel motor (Kraken X60) - Default: 25
        :type flywheel_motor_id: int
        :param hood_motor_id: CAN ID for the hood motor (Neo550) - Default: 26
        :type hood_motor_id: int
        """
        super().__init__()

        # Configuration variables
        self.FLYWHEEL_GEAR_RATIO = 2.0  # 2:1 gear ratio
        self.FLYWHEEL_DIAMETER_METERS = 0.127  # 5 inches in meters
        self.HOOD_GEAR_RATIO = 2.0  # 2:1 gear ratio

        # Hood angle limits (in degrees)
        self.MIN_HOOD_ANGLE = 0.0  # Flat/low shot
        self.MAX_HOOD_ANGLE = 60.0  # High arc shot

        # Flywheel target speeds (in rotations per second)
        self.FLYWHEEL_IDLE_SPEED = 0.0
        self.FLYWHEEL_SHOOT_SPEED = 60.0  # ~3600 RPM at mechanism

        # Motor controllers
        self.flywheel_motor = TalonFX(flywheel_motor_id)
        self.hood_motor = SparkMax(hood_motor_id, SparkMax.MotorType.kBrushless)
        self.hood_encoder = self.hood_motor.getAbsoluteEncoder()

        # Configure motors
        self._configure_flywheel_motor()
        self._configure_hood_motor()

        # PID Controllers (WPILib)
        self.flywheel_pid = PIDController(0.1, 0.0, 0.0)
        self.flywheel_pid.setTolerance(2.0)  # ±2 RPS tolerance

        self.hood_pid = PIDController(0.05, 0.0, 0.005)
        self.hood_pid.setTolerance(1.0)  # ±1 degree tolerance

        # Simulation
        # Create linear system plant for flywheel simulation
        flywheel_plant = LinearSystemId.flywheelSystem(
            DCMotor.krakenX60(1),
            0.02,  # MOI in kg*m^2 (estimate for flywheel)
            self.FLYWHEEL_GEAR_RATIO,
        )
        self.flywheel_sim = FlywheelSim(
            flywheel_plant,
            DCMotor.krakenX60(1),
        )

        self.hood_sim_physics = SingleJointedArmSim(
            DCMotor.NEO550(1),  # 1 Neo550 motor
            self.HOOD_GEAR_RATIO,
            0.01,  # MOI in kg*m^2
            0.15,  # Hood length in meters
            math.radians(self.MIN_HOOD_ANGLE),
            math.radians(self.MAX_HOOD_ANGLE),
            True,  # Simulate gravity
            math.radians(self.MIN_HOOD_ANGLE),  # Starting angle
        )
        self.hood_sim = SparkMaxSim(self.hood_motor, DCMotor.NEO550(1))
        self.flywheel_sim_state = self.flywheel_motor.sim_state

        # State variables
        self.target_flywheel_speed = 0.0  # RPS
        self.target_hood_angle = 30.0  # degrees (mid-range default)

        # NetworkTables setup
        nt = NetworkTableInstance.getDefault()
        input_prefix = "SimInputs" if RobotBase.isSimulation() else "RealInputs"
        output_prefix = "SimOutputs" if RobotBase.isSimulation() else "RealOutputs"
        self.nt_inputs = nt.getTable(f"{input_prefix}/Shooter")
        self.nt_outputs = nt.getTable(f"{output_prefix}/Shooter")

    def _configure_flywheel_motor(self):
        """Configure the flywheel motor with appropriate settings."""
        config = configs.TalonFXConfiguration()

        # Motor output configuration
        config.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        config.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # Current limits
        config.current_limits.stator_current_limit = 80
        config.current_limits.stator_current_limit_enable = True
        config.current_limits.supply_current_limit = 60
        config.current_limits.supply_current_limit_enable = True

        # # Onboard PID (commented out - using WPILib PID instead)
        # config.slot0.k_p = 0.1
        # config.slot0.k_i = 0.0
        # config.slot0.k_d = 0.0
        # config.slot0.k_v = 0.12  # Feedforward for velocity control

        # Feedback configuration - use motor's built-in encoder
        config.feedback.sensor_to_mechanism_ratio = self.FLYWHEEL_GEAR_RATIO

        self.flywheel_motor.configurator.apply(config)

    def _configure_hood_motor(self):
        """Configure the hood motor and encoder with appropriate settings."""
        config = SparkMaxConfig()
        
        # Idle mode (brake to hold position)
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        # Inversion
        config.inverted(False)

        # Current limits
        config.smartCurrentLimit(20)  # Neo550 rated for 20A continuous

        # Absolute encoder configuration
        # Set position conversion factor (rotations to degrees)
        config.absoluteEncoder.positionConversionFactor(
            360.0 / self.HOOD_GEAR_RATIO
        ).velocityConversionFactor(
            (360.0 / self.HOOD_GEAR_RATIO) / 60.0  # RPM to degrees per second
        ).zeroOffset(
            0.0  # Set zero offset if needed (in rotations, 0.0-1.0)
        ).inverted(
            False  # Invert encoder if needed
        )

        # # Onboard PID (commented out - using WPILib PID instead)
        # config.closedLoop.P(0.05).I(0.0).D(0.005).outputRange(-0.5, 0.5)

        # Apply configuration and persist to flash
        self.hood_motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

    def set_flywheel_speed(self, speed_rps: float):
        """
        Set the target flywheel speed.

        :param speed_rps: Target speed in rotations per second
        :type speed_rps: float
        """
        self.target_flywheel_speed = max(0.0, speed_rps)  # No negative speeds

    def set_hood_angle(self, angle_degrees: float):
        """
        Set the target hood angle.

        :param angle_degrees: Target angle in degrees
        :type angle_degrees: float
        """
        # Clamp to limits
        self.target_hood_angle = max(
            self.MIN_HOOD_ANGLE, min(self.MAX_HOOD_ANGLE, angle_degrees)
        )

    def get_flywheel_speed(self) -> float:
        """
        Get the current flywheel speed in rotations per second.

        :returns: Current speed in RPS
        :rtype: float
        """
        return self.flywheel_motor.get_velocity().value

    def get_hood_angle(self) -> float:
        """
        Get the current hood angle in degrees.

        :returns: Current angle in degrees
        :rtype: float
        """
        return self.hood_encoder.getPosition()

    def at_target_speed(self) -> bool:
        """
        Check if the flywheel is at target speed.

        :returns: True if at target within tolerance
        :rtype: bool
        """
        return self.flywheel_pid.atSetpoint()

    def at_target_hood_angle(self) -> bool:
        """
        Check if the hood is at target angle.

        :returns: True if at target within tolerance
        :rtype: bool
        """
        return self.hood_pid.atSetpoint()

    def ready_to_shoot(self) -> bool:
        """
        Check if shooter is ready (flywheel at speed and hood at angle).

        :returns: True if ready to shoot
        :rtype: bool
        """
        return self.at_target_speed() and self.at_target_hood_angle()

    def spin_up(self, speed_rps: float = None):
        """
        Spin up the flywheel to shooting speed.

        :param speed_rps: Optional custom speed, uses default if None
        :type speed_rps: float
        """
        if speed_rps is None:
            speed_rps = self.FLYWHEEL_SHOOT_SPEED
        self.set_flywheel_speed(speed_rps)

    def spin_down(self):
        """Stop the flywheel."""
        self.set_flywheel_speed(0.0)

    def set_low_shot(self):
        """Configure for a low-angle shot."""
        self.set_hood_angle(15.0)

    def set_mid_shot(self):
        """Configure for a mid-range shot."""
        self.set_hood_angle(30.0)

    def set_high_shot(self):
        """Configure for a high-arc shot."""
        self.set_hood_angle(50.0)

    def stop(self):
        """Stop all shooter motors."""
        self.spin_down()
        self.hood_motor.set(0.0)

    def periodic(self):
        """
        Called periodically by the scheduler.
        Updates motor outputs based on PID calculations and publishes telemetry.
        """
        # 1. Log all raw motor and encoder data
        self.nt_inputs.putNumber("FlywheelMotor/Position_rot", self.flywheel_motor.get_position().value)
        self.nt_inputs.putNumber("FlywheelMotor/Velocity_rps", self.flywheel_motor.get_velocity().value)
        self.nt_inputs.putNumber("FlywheelMotor/Temperature_C", self.flywheel_motor.get_device_temp().value)
        self.nt_inputs.putNumber("FlywheelMotor/Current_A", self.flywheel_motor.get_stator_current().value)
        self.nt_inputs.putNumber("FlywheelMotor/Voltage_V", self.flywheel_motor.get_motor_voltage().value)
        
        self.nt_inputs.putNumber("HoodMotor/Position_deg", self.hood_encoder.getPosition())
        self.nt_inputs.putNumber("HoodMotor/Velocity_dps", self.hood_encoder.getVelocity())
        self.nt_inputs.putNumber("HoodMotor/Temperature_C", self.hood_motor.getMotorTemperature())
        self.nt_inputs.putNumber("HoodMotor/Current_A", self.hood_motor.getOutputCurrent())
        self.nt_inputs.putNumber("HoodMotor/AppliedOutput", self.hood_motor.getAppliedOutput())
        self.nt_inputs.putNumber("HoodMotor/BusVoltage_V", self.hood_motor.getBusVoltage())

        # 2. Perform calculations (PID)
        current_flywheel_speed = self.get_flywheel_speed()
        current_hood_angle = self.get_hood_angle()
        
        flywheel_output = self.flywheel_pid.calculate(
            current_flywheel_speed, self.target_flywheel_speed
        )
        hood_output = self.hood_pid.calculate(
            current_hood_angle, self.target_hood_angle
        )
        clamped_hood_output = max(-0.3, min(0.3, hood_output))

        # 3. Apply motor outputs
        self.flywheel_motor.set(flywheel_output)
        self.hood_motor.set(clamped_hood_output)

        # 4. Log calculated/converted values
        self.nt_outputs.putNumber("FlywheelSpeed_rps", current_flywheel_speed)
        self.nt_outputs.putNumber("TargetFlywheelSpeed_rps", self.target_flywheel_speed)
        self.nt_outputs.putNumber("FlywheelPIDOutput", flywheel_output)
        self.nt_outputs.putBoolean("AtSpeed", self.at_target_speed())
        
        self.nt_outputs.putNumber("HoodAngle_deg", current_hood_angle)
        self.nt_outputs.putNumber("TargetHoodAngle_deg", self.target_hood_angle)
        self.nt_outputs.putNumber("HoodPIDOutput", hood_output)
        self.nt_outputs.putNumber("HoodClampedOutput", clamped_hood_output)
        self.nt_outputs.putBoolean("HoodAtTarget", self.at_target_hood_angle())
        
        self.nt_outputs.putBoolean("ReadyToShoot", self.ready_to_shoot())

    def simulationPeriodic(self):
        """Update simulation state."""
        # Update flywheel physics simulation
        self.flywheel_sim.setInput(np.array([self.flywheel_motor.get() * 12.0]))
        self.flywheel_sim.update(0.020)  # 20ms period
        
        # Feed flywheel simulation back to TalonFX sim_state
        flywheel_velocity_rps = self.flywheel_sim.getAngularVelocity() / (2 * math.pi)  # rad/s to RPS
        self.flywheel_sim_state.set_rotor_velocity(flywheel_velocity_rps)
        # Position integrates from velocity: add (velocity * dt) to current position
        self.flywheel_sim_state.add_rotor_position(flywheel_velocity_rps * 0.020)
        self.flywheel_sim_state.set_supply_voltage(12.0)

        # Update hood physics simulation
        self.hood_sim_physics.setInput(np.array([self.hood_motor.get() * 12.0]))
        self.hood_sim_physics.update(0.020)  # 20ms period
        
        # Update hood SparkMax simulation
        # Convert rad/s to degrees/s for the hood encoder
        hood_velocity_dps = math.degrees(self.hood_sim_physics.getVelocity())
        self.hood_sim.iterate(hood_velocity_dps, 12.0, 0.020)
