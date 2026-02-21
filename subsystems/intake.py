from commands2 import Subsystem
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6 import configs, signals, units
from wpimath.controller import PIDController
from wpilib import RobotBase
from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor
from ntcore import NetworkTableInstance
import math
import numpy as np


class Intake(Subsystem):
    """
    Intake subsystem with pivoting axis and rolling bar.
    Uses a Kraken X60 motor for pivot control and a CANCoder for position feedback.
    """

    def __init__(
        self,
        pivot_motor_id: int = 20,
        roller_motor_id: int = 21,
        cancoder_id: int = 20,
    ):
        """
        Initialize the Intake subsystem.

        :param pivot_motor_id: CAN ID for the pivot motor (Kraken X60) - Default: 20
        :type pivot_motor_id: int
        :param roller_motor_id: CAN ID for the roller motor (Kraken X60) - Default: 21
        :type roller_motor_id: int
        :param cancoder_id: CAN ID for the CANcoder (matches pivot motor) - Default: 20
        :type cancoder_id: int
        """
        super().__init__()

        # Configuration variables
        self.GEAR_RATIO = 2.0  # 2:1 gear ratio
        self.PIVOT_LENGTH_METERS = 0.127  # 5 inches in meters
        self.ROLLER_GEAR_RATIO = 2.0  # 2:1 gear ratio for roller

        # Pivot angle limits (in degrees)
        self.MIN_ANGLE = 0.0  # Fully retracted
        self.MAX_ANGLE = 90.0  # Fully extended

        # Motor controllers
        self.pivot_motor = TalonFX(pivot_motor_id)
        self.roller_motor = TalonFX(roller_motor_id)
        self.encoder = CANcoder(cancoder_id)

        # Configure motors
        self._configure_pivot_motor()
        self._configure_roller_motor()
        self._configure_encoder()

        # PID Controllers (WPILib)
        self.pivot_pid = PIDController(0.1, 0.0, 0.0)
        self.pivot_pid.setTolerance(2.0)  # 2 degree tolerance

        # Simulation
        self.pivot_sim = SingleJointedArmSim(
            DCMotor.krakenX60(1),  # 1 Kraken motor
            self.GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(self.PIVOT_LENGTH_METERS, 5.0),  # 5kg mass
            self.PIVOT_LENGTH_METERS,
            math.radians(self.MIN_ANGLE),
            math.radians(self.MAX_ANGLE),
            True,  # Simulate gravity
            math.radians(self.MIN_ANGLE),  # Starting angle
        )
        self.pivot_sim_state = self.pivot_motor.sim_state
        self.roller_sim_state = self.roller_motor.sim_state
        self.encoder_sim_state = self.encoder.sim_state

        # State variables
        self.target_angle = self.MIN_ANGLE
        self.roller_speed = 0.0

        # NetworkTables setup
        nt = NetworkTableInstance.getDefault()
        input_prefix = "SimInputs" if RobotBase.isSimulation() else "RealInputs"
        output_prefix = "SimOutputs" if RobotBase.isSimulation() else "RealOutputs"
        self.nt_inputs = nt.getTable(f"{input_prefix}/Intake")
        self.nt_outputs = nt.getTable(f"{output_prefix}/Intake")

    def _configure_pivot_motor(self):
        """Configure the pivot motor with appropriate settings."""
        config = configs.TalonFXConfiguration()

        # Motor output configuration
        config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        config.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # Current limits
        config.current_limits.stator_current_limit = 40
        config.current_limits.stator_current_limit_enable = True
        config.current_limits.supply_current_limit = 30
        config.current_limits.supply_current_limit_enable = True

        # # Onboard PID (commented out - using WPILib PID instead)
        # config.slot0.k_p = 0.1
        # config.slot0.k_i = 0.0
        # config.slot0.k_d = 0.0
        # config.slot0.k_v = 0.0

        # Feedback configuration
        config.feedback.feedback_remote_sensor_id = self.encoder.device_id
        config.feedback.feedback_sensor_source = (
            signals.FeedbackSensorSourceValue.FUSED_CANCODER
        )
        config.feedback.sensor_to_mechanism_ratio = self.GEAR_RATIO
        config.feedback.rotor_to_sensor_ratio = 1.0

        self.pivot_motor.configurator.apply(config)

    def _configure_roller_motor(self):
        """Configure the roller motor with appropriate settings."""
        config = configs.TalonFXConfiguration()

        # Motor output configuration
        config.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        config.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # Current limits
        config.current_limits.stator_current_limit = 60
        config.current_limits.stator_current_limit_enable = True
        config.current_limits.supply_current_limit = 40
        config.current_limits.supply_current_limit_enable = True

        self.roller_motor.configurator.apply(config)

    def _configure_encoder(self):
        """Configure the CANcoder."""
        config = configs.CANcoderConfiguration()

        # Set magnet offset if needed
        config.magnet_sensor.magnet_offset = 0.0
        config.magnet_sensor.absolute_sensor_discontinuity_point = 1

        self.encoder.configurator.apply(config)

    def set_pivot_angle(self, angle_degrees: float):
        """
        Set the target pivot angle.

        :param angle_degrees: Target angle in degrees
        :type angle_degrees: float
        """
        # Clamp to limits
        self.target_angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, angle_degrees))

    def set_roller_speed(self, speed: float):
        """
        Set the roller speed.

        :param speed: Speed from -1.0 to 1.0 (positive = intake)
        :type speed: float
        """
        self.roller_speed = max(-1.0, min(1.0, speed))

    def get_pivot_angle(self) -> float:
        """
        Get the current pivot angle in degrees.

        :returns: Current angle in degrees
        :rtype: float
        """
        return self.encoder.get_absolute_position().value * 360.0

    def at_target_angle(self) -> bool:
        """
        Check if the pivot is at the target angle.

        :returns: True if at target within tolerance
        :rtype: bool
        """
        return self.pivot_pid.atSetpoint()

    def stow(self):
        """Retract the intake to the stowed position."""
        self.set_pivot_angle(self.MIN_ANGLE)
        self.set_roller_speed(0.0)

    def deploy(self):
        """Deploy the intake to the extended position."""
        self.set_pivot_angle(self.MAX_ANGLE)

    def intake_in(self, speed: float = 0.8):
        """
        Run the roller to intake game pieces.

        :param speed: Intake speed (0.0 to 1.0)
        :type speed: float
        """
        self.set_roller_speed(abs(speed))

    def intake_out(self, speed: float = 0.5):
        """
        Run the roller to eject game pieces.

        :param speed: Eject speed (0.0 to 1.0)
        :type speed: float
        """
        self.set_roller_speed(-abs(speed))

    def stop_roller(self):
        """Stop the roller motor."""
        self.set_roller_speed(0.0)

    def periodic(self):
        """
        Called periodically by the scheduler.
        Updates motor outputs based on PID calculations and publishes telemetry.
        """
        # 1. Log all raw motor and encoder data
        self.nt_inputs.putNumber("PivotMotor/Position_rot", self.pivot_motor.get_position().value)
        self.nt_inputs.putNumber("PivotMotor/Velocity_rps", self.pivot_motor.get_velocity().value)
        self.nt_inputs.putNumber("PivotMotor/Temperature_C", self.pivot_motor.get_device_temp().value)
        self.nt_inputs.putNumber("PivotMotor/Current_A", self.pivot_motor.get_stator_current().value)
        self.nt_inputs.putNumber("PivotMotor/Voltage_V", self.pivot_motor.get_motor_voltage().value)
        
        self.nt_inputs.putNumber("RollerMotor/Velocity_rps", self.roller_motor.get_velocity().value)
        self.nt_inputs.putNumber("RollerMotor/Temperature_C", self.roller_motor.get_device_temp().value)
        self.nt_inputs.putNumber("RollerMotor/Current_A", self.roller_motor.get_stator_current().value)
        
        self.nt_inputs.putNumber("CANcoder/Position_rot", self.encoder.get_absolute_position().value)

        # 2. Perform calculations (PID)
        current_angle = self.get_pivot_angle()
        pid_output = self.pivot_pid.calculate(current_angle, self.target_angle)

        # 3. Apply motor outputs
        self.pivot_motor.set(pid_output)
        self.roller_motor.set(self.roller_speed)

        # 4. Log calculated/converted values
        self.nt_outputs.putNumber("CurrentAngle_deg", current_angle)
        self.nt_outputs.putNumber("TargetAngle_deg", self.target_angle)
        self.nt_outputs.putNumber("PIDOutput", pid_output)
        self.nt_outputs.putNumber("RollerSpeedCommand", self.roller_speed)
        self.nt_outputs.putBoolean("AtTarget", self.at_target_angle())

    def simulationPeriodic(self):
        """Update simulation state."""
        # Update arm physics simulation
        self.pivot_sim.setInput(np.array([self.pivot_motor.get() * 12.0]))
        self.pivot_sim.update(0.020)  # 20ms period
        
        # Feed simulation back to TalonFX and CANcoder sim_state
        # Convert radians to rotations for motor position
        sim_angle_rot = self.pivot_sim.getAngle() / (2 * math.pi)
        sim_velocity_rps = self.pivot_sim.getVelocity() / (2 * math.pi)
        
        # Update pivot motor position accounting for gearing
        self.pivot_sim_state.set_rotor_velocity(sim_velocity_rps * self.GEAR_RATIO)
        self.pivot_sim_state.set_raw_rotor_position(sim_angle_rot * self.GEAR_RATIO)
        self.pivot_sim_state.set_supply_voltage(12.0)
        
        # Update CANcoder position (in rotations, 0.0-1.0)
        self.encoder_sim_state.set_raw_position(sim_angle_rot % 1.0)
        self.encoder_sim_state.set_supply_voltage(12.0)
        
        # Update roller motor simulation (simple velocity)
        roller_velocity_rps = self.roller_motor.get() * 80.0  # Arbitrary max RPS
        self.roller_sim_state.set_rotor_velocity(roller_velocity_rps)
        self.roller_sim_state.add_rotor_position(roller_velocity_rps * 0.020)
        self.roller_sim_state.set_supply_voltage(12.0)
