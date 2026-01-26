import math
from typing import Callable
from commands2 import Subsystem
from wpilib import SmartDashboard, RobotBase, Mechanism2d, MechanismLigament2d, MechanismObject2d, Color, Color8Bit
from ntcore import NetworkTableInstance
from wpimath.units import degrees, turns, rotationsToDegrees, degreesToRotations, inchesToMeters
from wpimath.geometry import Rotation2d, Translation2d, Pose2d, Transform2d
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

    _robotPosition:Translation2d = Translation2d(0, 0)
    _desiredPosition:degrees = 0

    __getRobotPose:Callable[[],Pose2d] = lambda: Pose2d()

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

        # Robot Relative Position Data
        self._robotPosition = Translation2d( inchesToMeters(8), inchesToMeters(8) )
                                          
        self._logger = NetworkTableInstance.getDefault().getTable("000Turret")
        self._logPoseRelative = self._logger.getStructTopic( "PoseRelative", Pose2d ).publish()
        self._logPoseAbsolute = self._logger.getStructTopic( "PoseAbsolute", Pose2d ).publish()

        self.__kraken = DCMotor.krakenX60(1)

    def periodic(self):
        self._motor.set_control(
            PositionDutyCycle(
                degreesToRotations( self.getDesiredPosition() )
            ).with_override_brake_dur_neutral( True )
        )

        self._logPoseRelative.set( self.getRelativePose() )
        self._logPoseAbsolute.set( self.getAbsolutePose() )
        # self._logMech.set( self._mech )

    def simulationPeriodic(self):
        motorRps = self._motor.get() * ( self.__kraken.freeSpeed / ( 2 * math.pi ) )
        sensorRps = motorRps / self._motorToSensorRatio

        self._motor.sim_state.set_rotor_velocity( motorRps )
        self._motor.sim_state.add_rotor_position( motorRps * 0.02 )

        self._sensor.sim_state.set_velocity( sensorRps )
        self._sensor.sim_state.add_position( sensorRps * 0.02 )

    ### BEGIN: CROSS-SYSTEM INTERFACE FUNCTIONS ###       
    def setRobotPoseGetter(self, func: Callable[[], Pose2d]) -> None:
        self.__getRobotPose = func

    ### BEGIN: POSITION BASED FUNCTIONS ###
    def setPosition(self, position: degrees, fieldRelative: bool = False) -> None:
        if fieldRelative:
            position = position - self.__getRobotPose().rotation().degrees()

        self._desiredPosition = position
        # self._motor.set_control( PositionDutyCycle( degreesToRotations( self.getDesiredPosition() ) ) )

    def getPosition(self) -> degrees:
        return self.getDegrees()

    def getDesiredPosition(self) -> degrees:
        return self._desiredPosition
    
    def atPosition(self, tolerance: degrees = 1.0 ) -> bool:
        diff = self.getDegrees() - self.getDesiredPosition()
        return abs(diff) <= tolerance

    ### BEGIN: UNIT CONVERSION BASED FUNCTIONS ###
    def getDegrees(self) -> degrees:
        return rotationsToDegrees( self.getRotations() )

    def getRotations(self) -> turns:
        return self._sensor.get_absolute_position().value_as_double
    
    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromRotations( self.getRotations() )
   
    ### BEGIN: WPILIB GEOMETRY FUNCTIONS ###
    def getAbsolutePose(self) -> Pose2d:
        robotPose = self.__getRobotPose()
        transformPose = Transform2d( self._robotPosition, self.getRotation2d() )
        return robotPose.transformBy( transformPose )

    def getRelativePose(self) -> Pose2d:
        return Pose2d( self._robotPosition, self.getRotation2d() )

    