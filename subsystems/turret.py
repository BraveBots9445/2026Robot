import math
from commands2 import Subsystem
from wpilib import SmartDashboard, RobotBase
from ntcore import NetworkTableInstance
from wpimath.units import degrees, rotationsToDegrees, degreesToRotations
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.system.plant import DCMotor
from wpilib.simulation import DCMotorSim
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.controls import * # PositionVoltage, DutyCycleOut
from phoenix6.signals.spn_enums import * # NeutralModeValue, InvertedValue, SensorDirectionValue
from phoenix6.hardware import CANcoder
from phoenix6 import SignalLogger

class Turret(Subsystem):
    _motor:TalonFX
    _sensor:CANcoder
    _motorToSensorRatio:float = 10.0

    _desiredPosition:degrees = 0

    def __init__(self):
        # Encoder Config
        ## Config
        encoderCfg = CANcoderConfiguration()
        encoderCfg.magnet_sensor.absolute_sensor_discontinuity_point = 0.5
        encoderCfg.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        if not RobotBase.isSimulation(): encoderCfg.magnet_sensor.magnet_offset = 0.125
        ## Init
        self._sensor = CANcoder(22)
        self._sensor.configurator.apply( encoderCfg )
        self._sensor.set_position( self._sensor.get_absolute_position().value )

        # Motor Config
        ## Config
        motorCfg = TalonFXConfiguration()
        motorCfg.motor_output.neutral_mode = NeutralModeValue.COAST
        motorCfg.current_limits.with_stator_current_limit( 40.0 ).with_stator_current_limit_enable( True )
        motorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        ## Feedback Sensor
        motorCfg.feedback.feedback_remote_sensor_id = self._sensor.device_id
        motorCfg.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        ## PID config
        motorCfg.slot0.k_p = 1.0
        motorCfg.slot0.k_i = 0.0
        motorCfg.slot0.k_d = 0.0
        motorCfg.slot0.k_v = 0.0
        ## Init
        self._motor = TalonFX(21)
        self._motor.configurator.apply( motorCfg )
        self._motorControl = PositionVoltage( 0 )

        self.setPosition( rotationsToDegrees( self._sensor.get_absolute_position().value_as_double ) )

        self._logger = NetworkTableInstance.getDefault().getTable("000Turret")
        self._topic = self._logger.getStructTopic( "000Turret/Rotation", Rotation2d )

    def periodic(self):
        self._motor.set_control( PositionDutyCycle( degreesToRotations( self.getDesiredPosition() ) ) )
        self._topic.publish().set( self.getRotation() )
        self._logger.getStructTopic( "000Turret/Rotation", Rotation2d ).publish().set( self.getRotation() )

    def simulationPeriodic(self):
        kraken = DCMotor.krakenX60(1)
        motorRps = self._motor.get() * ( kraken.freeSpeed / ( 2 * math.pi ) )
        sensorRps = motorRps / self._motorToSensorRatio

        self._motor.sim_state.set_rotor_velocity( motorRps )
        self._motor.sim_state.add_rotor_position( motorRps * 0.02 )

        self._sensor.sim_state.set_velocity( sensorRps )
        self._sensor.sim_state.add_position( sensorRps * 0.02 )
        ...

    def getRotation(self) -> Rotation2d:
        return Rotation2d().fromDegrees( self.getPosition() )

    def getPosition(self) -> degrees:
        rotations = self._sensor.get_absolute_position().value_as_double
        return rotationsToDegrees( rotations )
    
    def setPosition(self, position: degrees) -> None:
        self._desiredPosition = position

    def getDesiredPosition(self) -> degrees:
        return self._desiredPosition
    
    def atPosition(self, tolerance: degrees = 1.0 ) -> bool:
        diff = self.getPosition() - self.getDesiredPosition()
        return abs(diff) <= tolerance