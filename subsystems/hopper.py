from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6 import configs, signals
from wpilib import RobotBase
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor, LinearSystemId
from ntcore import NetworkTableInstance
import numpy as np
from tools.TelemetryService import get_telemetry_service


class Hopper(Subsystem):
    """
    Hopper subsystem with washing machine style turnstile and roller feed.
    Uses Kraken X60 motors for both the turnstile and feed roller.
    """

    def __init__(
        self,
        turnstile_motor_id: int = 22,
        feed_roller_motor_id: int = 23,
    ):
        """
        Initialize the Hopper subsystem.

        :param turnstile_motor_id: CAN ID for the turnstile motor (Kraken X60) - Default: 22
        :type turnstile_motor_id: int
        :param feed_roller_motor_id: CAN ID for the feed roller motor (Kraken X60) - Default: 23
        :type feed_roller_motor_id: int
        """
        super().__init__()

        # Configuration variables
        self.TURNSTILE_GEAR_RATIO = 2.0  # 2:1 gear ratio
        self.FEED_ROLLER_GEAR_RATIO = 2.0  # 2:1 gear ratio

        # Speed presets
        self.TURNSTILE_SPEED = 0.4  # Default turnstile speed
        self.FEED_SPEED = 0.6  # Default feed roller speed

        # Motor controllers
        self.turnstile_motor = TalonFX(turnstile_motor_id)
        self.feed_roller_motor = TalonFX(feed_roller_motor_id)

        # Configure motors
        self._configure_turnstile_motor()
        self._configure_feed_roller_motor()

        # Simulation - Create Linear System plants for FlywheelSim
        turnstile_plant = LinearSystemId.flywheelSystem(
            DCMotor.krakenX60(1),
            0.01,  # MOI in kg*m^2 (estimate for turnstile)
            self.TURNSTILE_GEAR_RATIO,
        )
        self.turnstile_sim = FlywheelSim(
            turnstile_plant,
            DCMotor.krakenX60(1),
        )

        feed_roller_plant = LinearSystemId.flywheelSystem(
            DCMotor.krakenX60(1),
            0.005,  # MOI in kg*m^2 (estimate for roller)
            self.FEED_ROLLER_GEAR_RATIO,
        )
        self.feed_roller_sim = FlywheelSim(
            feed_roller_plant,
            DCMotor.krakenX60(1),
        )
        self.turnstile_sim_state = self.turnstile_motor.sim_state
        self.feed_roller_sim_state = self.feed_roller_motor.sim_state

        # State variables
        self.turnstile_speed = 0.0
        self.feed_roller_speed = 0.0

        # NetworkTables setup
        nt = NetworkTableInstance.getDefault()
        input_prefix = "SimInputs" if RobotBase.isSimulation() else "RealInputs"
        output_prefix = "SimOutputs" if RobotBase.isSimulation() else "RealOutputs"
        self.nt_inputs = nt.getTable(f"{input_prefix}/Hopper")
        self.nt_outputs = nt.getTable(f"{output_prefix}/Hopper")

        # Telemetry cache and service registration
        self._telemetry_cache = {}
        get_telemetry_service().register_subsystem("Hopper", self._publish_telemetry)

    def _configure_turnstile_motor(self):
        """Configure the turnstile motor with appropriate settings."""
        config = configs.TalonFXConfiguration()

        # Motor output configuration
        config.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        config.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # Current limits
        config.current_limits.stator_current_limit = 40
        config.current_limits.stator_current_limit_enable = True
        config.current_limits.supply_current_limit = 30
        config.current_limits.supply_current_limit_enable = True

        # Feedback configuration for gear ratio
        config.feedback.sensor_to_mechanism_ratio = self.TURNSTILE_GEAR_RATIO

        self.turnstile_motor.configurator.apply(config)

    def _configure_feed_roller_motor(self):
        """Configure the feed roller motor with appropriate settings."""
        config = configs.TalonFXConfiguration()

        # Motor output configuration
        config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        config.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # Current limits
        config.current_limits.stator_current_limit = 60
        config.current_limits.stator_current_limit_enable = True
        config.current_limits.supply_current_limit = 40
        config.current_limits.supply_current_limit_enable = True

        # Feedback configuration for gear ratio
        config.feedback.sensor_to_mechanism_ratio = self.FEED_ROLLER_GEAR_RATIO

        self.feed_roller_motor.configurator.apply(config)

    def set_turnstile_speed(self, speed: float):
        """
        Set the turnstile speed.

        :param speed: Speed from -1.0 to 1.0 (positive = forward feed)
        :type speed: float
        """
        self.turnstile_speed = max(-1.0, min(1.0, speed))

    def set_feed_roller_speed(self, speed: float):
        """
        Set the feed roller speed.

        :param speed: Speed from -1.0 to 1.0 (positive = feed to shooter)
        :type speed: float
        """
        self.feed_roller_speed = max(-1.0, min(1.0, speed))

    def feed_to_shooter(self):
        """Run both turnstile and feed roller to feed game piece to shooter."""
        self.set_turnstile_speed(self.TURNSTILE_SPEED)
        self.set_feed_roller_speed(self.FEED_SPEED)

    def intake_from_floor(self):
        """Run turnstile to accept game piece from intake (reverse feed roller)."""
        self.set_turnstile_speed(self.TURNSTILE_SPEED)
        self.set_feed_roller_speed(-0.3)  # Slow reverse to accept

    def reverse_all(self):
        """Reverse both motors to eject game piece."""
        self.set_turnstile_speed(-self.TURNSTILE_SPEED)
        self.set_feed_roller_speed(-self.FEED_SPEED)

    def stop(self):
        """Stop all hopper motors."""
        self.set_turnstile_speed(0.0)
        self.set_feed_roller_speed(0.0)

    def get_turnstile_velocity(self) -> float:
        """
        Get the current turnstile velocity in rotations per second.

        :returns: Velocity in rotations per second
        :rtype: float
        """
        return self.turnstile_motor.get_velocity().value

    def get_feed_roller_velocity(self) -> float:
        """
        Get the current feed roller velocity in rotations per second.

        :returns: Velocity in rotations per second
        :rtype: float
        """
        return self.feed_roller_motor.get_velocity().value

    def periodic(self):
        """
        Called periodically by the scheduler (every 20ms).
        Updates motor outputs.
        Telemetry is cached here and published every 100ms by TelemetryService.
        """
        # 1. Read all sensor data ONCE per cycle
        turnstile_position = self.turnstile_motor.get_position().value
        turnstile_velocity = self.turnstile_motor.get_velocity().value
        turnstile_temp = self.turnstile_motor.get_device_temp().value
        turnstile_current = self.turnstile_motor.get_stator_current().value
        turnstile_voltage = self.turnstile_motor.get_motor_voltage().value
        
        feed_position = self.feed_roller_motor.get_position().value
        feed_velocity = self.feed_roller_motor.get_velocity().value
        feed_temp = self.feed_roller_motor.get_device_temp().value
        feed_current = self.feed_roller_motor.get_stator_current().value
        feed_voltage = self.feed_roller_motor.get_motor_voltage().value
        
        # 2. Apply motor outputs (control loop runs at full 20ms rate)
        self.turnstile_motor.set(self.turnstile_speed)
        self.feed_roller_motor.set(self.feed_roller_speed)

        # 3. Cache telemetry data (published every 100ms on separate thread)
        # Use the SAME sensor readings we already collected
        self._telemetry_cache = {
            "turnstile_position": turnstile_position,
            "turnstile_velocity": turnstile_velocity,
            "turnstile_temp": turnstile_temp,
            "turnstile_current": turnstile_current,
            "turnstile_voltage": turnstile_voltage,
            "feed_position": feed_position,
            "feed_velocity": feed_velocity,
            "feed_temp": feed_temp,
            "feed_current": feed_current,
            "feed_voltage": feed_voltage,
            "turnstile_speed_cmd": self.turnstile_speed,
            "feed_speed_cmd": self.feed_roller_speed,
        }
    
    def _publish_telemetry(self):
        """
        Called by TelemetryService every 100ms on separate thread.
        Publishes cached telemetry data to NetworkTables.
        """
        if not self._telemetry_cache:
            return
        
        cache = self._telemetry_cache
        
        # Publish inputs
        self.nt_inputs.putNumber("TurnstileMotor/Position_rot", cache["turnstile_position"])
        self.nt_inputs.putNumber("TurnstileMotor/Velocity_rps", cache["turnstile_velocity"])
        self.nt_inputs.putNumber("TurnstileMotor/Temperature_C", cache["turnstile_temp"])
        self.nt_inputs.putNumber("TurnstileMotor/Current_A", cache["turnstile_current"])
        self.nt_inputs.putNumber("TurnstileMotor/Voltage_V", cache["turnstile_voltage"])
        self.nt_inputs.putNumber("FeedMotor/Position_rot", cache["feed_position"])
        self.nt_inputs.putNumber("FeedMotor/Velocity_rps", cache["feed_velocity"])
        self.nt_inputs.putNumber("FeedMotor/Temperature_C", cache["feed_temp"])
        self.nt_inputs.putNumber("FeedMotor/Current_A", cache["feed_current"])
        self.nt_inputs.putNumber("FeedMotor/Voltage_V", cache["feed_voltage"])
        
        # Publish outputs
        self.nt_outputs.putNumber("TurnstileSpeedCommand", cache["turnstile_speed_cmd"])
        self.nt_outputs.putNumber("FeedRollerSpeedCommand", cache["feed_speed_cmd"])
        self.nt_outputs.putNumber("TurnstileVelocity_rps", cache["turnstile_velocity"])
        self.nt_outputs.putNumber("FeedRollerVelocity_rps", cache["feed_velocity"])

    def simulationPeriodic(self):
        """Update simulation state."""
        # Update turnstile physics simulation
        self.turnstile_sim.setInput(np.array([self.turnstile_motor.get() * 12.0]))
        self.turnstile_sim.update(0.020)  # 20ms period
        
        # Feed simulation back to TalonFX sim_state
        # Convert rad/s to RPS
        turnstile_velocity_rps = self.turnstile_sim.getAngularVelocity() / (2 * 3.14159)
        self.turnstile_sim_state.set_rotor_velocity(turnstile_velocity_rps)
        self.turnstile_sim_state.add_rotor_position(turnstile_velocity_rps * 0.020)
        self.turnstile_sim_state.set_supply_voltage(12.0)

        # Update feed roller physics simulation
        self.feed_roller_sim.setInput(np.array([self.feed_roller_motor.get() * 12.0]))
        self.feed_roller_sim.update(0.020)  # 20ms period
        
        # Feed simulation back to TalonFX sim_state
        feed_velocity_rps = self.feed_roller_sim.getAngularVelocity() / (2 * 3.14159)
        self.feed_roller_sim_state.set_rotor_velocity(feed_velocity_rps)
        self.feed_roller_sim_state.add_rotor_position(feed_velocity_rps * 0.020)
        self.feed_roller_sim_state.set_supply_voltage(12.0)
