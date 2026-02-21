from commands2 import Subsystem
from rev import SparkMax, SparkMaxConfig, ResetMode, PersistMode, SparkMaxSim
from wpimath.controller import PIDController
from wpilib import RobotBase
from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor
from ntcore import NetworkTableInstance
import math
import numpy as np


class Turret(Subsystem):
    """
    Turret subsystem with 360-degree rotation capability.
    Uses a Neo550 motor for rotation and ThroughBore absolute encoder for position feedback.
    Aiming range: -180° to +180°
    """

    def __init__(
        self,
        motor_id: int = 24,
    ):
        """
        Initialize the Turret subsystem.

        :param motor_id: CAN ID for the turret motor (Neo550) - Default: 24
        :type motor_id: int
        """
        super().__init__()

        # Configuration variables
        self.GEAR_RATIO = 2.0  # 2:1 gear ratio
        
        # Angle limits (in degrees, relative to robot front)
        self.MIN_ANGLE = -180.0  # Full left
        self.MAX_ANGLE = 180.0  # Full right

        # Motor controller (Neo550)
        self.motor = SparkMax(motor_id, SparkMax.MotorType.kBrushless)
        
        # ThroughBore absolute encoder
        self.encoder = self.motor.getAbsoluteEncoder()

        # Configure motor and encoder
        self._configure_motor()

        # PID Controllers (WPILib)
        self.pid = PIDController(0.05, 0.0, 0.005)
        self.pid.setTolerance(1.0)  # 1 degree tolerance
        self.pid.enableContinuousInput(-180.0, 180.0)  # Wraps around at ±180°

        # Simulation
        self.sim_physics = SingleJointedArmSim(
            DCMotor.NEO550(1),  # 1 Neo550 motor
            self.GEAR_RATIO,
            0.1,  # MOI in kg*m^2 (estimate for turret rotating mass)
            0.0,  # Arm length (turret rotates in place, not gravity-affected)
            math.radians(self.MIN_ANGLE),
            math.radians(self.MAX_ANGLE),
            False,  # No gravity for rotating turret
            math.radians(0.0),  # Starting angle (forward)
        )
        self.sim = SparkMaxSim(self.motor, DCMotor.NEO550(1))

        # State variables
        self.target_angle = 0.0  # Start facing forward

        # NetworkTables setup
        nt = NetworkTableInstance.getDefault()
        input_prefix = "SimInputs" if RobotBase.isSimulation() else "RealInputs"
        output_prefix = "SimOutputs" if RobotBase.isSimulation() else "RealOutputs"
        self.nt_inputs = nt.getTable(f"{input_prefix}/Turret")
        self.nt_outputs = nt.getTable(f"{output_prefix}/Turret")

    def _configure_motor(self):
        """Configure the turret motor and encoder with appropriate settings."""
        config = SparkMaxConfig()
        
        # Idle mode (brake to hold position)
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        # Inversion
        config.inverted(False)

        # Current limits
        config.smartCurrentLimit(20)  # Neo550 rated for 20A continuous

        # Absolute encoder configuration
        # Set position conversion factor (rotations to degrees)
        # After gearing: 1 motor rotation = 1/GEAR_RATIO turret rotations
        # 1 turret rotation = 360 degrees
        config.absoluteEncoder.positionConversionFactor(
            360.0 / self.GEAR_RATIO
        ).velocityConversionFactor(
            (360.0 / self.GEAR_RATIO) / 60.0  # RPM to degrees per second
        ).zeroOffset(
            0.0  # Set zero offset if needed (in rotations, 0.0-1.0)
        ).inverted(
            False  # Invert encoder if needed
        )

        # # Onboard PID (commented out - using WPILib PID instead)
        # config.closedLoop.P(0.05).I(0.0).D(0.005).outputRange(-1.0, 1.0)

        # Apply configuration and persist to flash
        self.motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

    def set_target_angle(self, angle_degrees: float):
        """
        Set the target turret angle.

        :param angle_degrees: Target angle in degrees (-180 to +180, 0 = robot front)
        :type angle_degrees: float
        """
        # Normalize angle to -180 to +180 range
        angle_degrees = ((angle_degrees + 180.0) % 360.0) - 180.0
        
        # Clamp to limits (though PID continuous input handles wrapping)
        self.target_angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, angle_degrees))

    def get_angle(self) -> float:
        """
        Get the current turret angle in degrees.

        :returns: Current angle in degrees (-180 to +180)
        :rtype: float
        """
        angle = self.encoder.getPosition()
        # Normalize to -180 to +180
        return ((angle + 180.0) % 360.0) - 180.0

    def get_velocity(self) -> float:
        """
        Get the current turret velocity in degrees per second.

        :returns: Velocity in degrees per second
        :rtype: float
        """
        return self.encoder.getVelocity()

    def at_target_angle(self) -> bool:
        """
        Check if the turret is at the target angle.

        :returns: True if at target within tolerance
        :rtype: bool
        """
        return self.pid.atSetpoint()

    def aim_forward(self):
        """Aim the turret to face the front of the robot (0°)."""
        self.set_target_angle(0.0)

    def aim_backward(self):
        """Aim the turret to face the back of the robot (180°)."""
        self.set_target_angle(180.0)

    def aim_left(self):
        """Aim the turret to face left (90°)."""
        self.set_target_angle(90.0)

    def aim_right(self):
        """Aim the turret to face right (-90°)."""
        self.set_target_angle(-90.0)

    def stop(self):
        """Stop turret movement (holds current position via brake mode)."""
        self.motor.set(0.0)

    def periodic(self):
        """
        Called periodically by the scheduler.
        Updates motor outputs based on PID calculations and publishes telemetry.
        """
        # 1. Log all raw motor and encoder data
        self.nt_inputs.putNumber("Motor/Position_deg", self.encoder.getPosition())
        self.nt_inputs.putNumber("Motor/Velocity_dps", self.encoder.getVelocity())
        self.nt_inputs.putNumber("Motor/Temperature_C", self.motor.getMotorTemperature())
        self.nt_inputs.putNumber("Motor/Current_A", self.motor.getOutputCurrent())
        self.nt_inputs.putNumber("Motor/AppliedOutput", self.motor.getAppliedOutput())
        self.nt_inputs.putNumber("Motor/BusVoltage_V", self.motor.getBusVoltage())

        # 2. Perform calculations (PID)
        current_angle = self.get_angle()
        pid_output = self.pid.calculate(current_angle, self.target_angle)
        clamped_output = max(-0.5, min(0.5, pid_output))

        # 3. Apply motor output
        self.motor.set(clamped_output)

        # 4. Log calculated/converted values
        self.nt_outputs.putNumber("CurrentAngle_deg", current_angle)
        self.nt_outputs.putNumber("TargetAngle_deg", self.target_angle)
        self.nt_outputs.putNumber("Velocity_dps", self.get_velocity())
        self.nt_outputs.putNumber("PIDOutput", pid_output)
        self.nt_outputs.putNumber("ClampedOutput", clamped_output)
        self.nt_outputs.putBoolean("AtTarget", self.at_target_angle())

    def simulationPeriodic(self):
        """Update simulation state."""
        # Update turret physics simulation
        self.sim_physics.setInput(np.array([self.motor.get() * 12.0]))
        self.sim_physics.update(0.020)  # 20ms period
        
        # Update SparkMax simulation with velocity from physics
        # Convert rad/s to degrees/s
        turret_velocity_dps = math.degrees(self.sim_physics.getVelocity())
        self.sim.iterate(turret_velocity_dps, 12.0, 0.020)
