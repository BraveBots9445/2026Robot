from commands2 import Subsystem
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations, degrees, inchesToMeters, degreesToRotations
from wpimath.controller import PIDController
from wpilib import SmartDashboard
from wpilib.simulation import SingleJointedArmSim
from ntcore import NetworkTableInstance

from phoenix6 import configs, signals
from phoenix6.hardware import TalonFX,CANcoder




class Intake(Subsystem):
    simTalon = DCMotor.krakenX60()

    def __init__(self,):
        self.nettable = NetworkTableInstance.getDefault().getTable("000Intake")
        self.pivotmotor = TalonFX(20)
        self.rollermotor = TalonFX(21)
        self.pivotsetpoint = Rotation2d(90)
        self.speed = 0
        self.cancoderid = 20
        self.cancoder = CANcoder(self.cancoderid)
        self.pivotPID = PIDController(Kp=.10, Ki=0, Kd=0)
        talonfxconfigurator = self.pivotmotor.configurator
        SmartDashboard.putData(self.pivotPID)
        SmartDashboard.putData(self)
        self._gearRatio = 9 / 1
        self._armSim = SingleJointedArmSim(self.simTalon, self._gearRatio, 0.0161, inchesToMeters(12.5), -float('inf'), float('inf'), True, 0)\
        
        fx_cfg = configs.TalonFXConfiguration()
        fx_cfg.feedback.feedback_remote_sensor_id = self.cancoderid
        fx_cfg.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.REMOTE_CANCODER
        limit_configs = configs.CurrentLimitsConfigs()
        limit_configs.stator_current_limit = 120
        limit_configs.stator_current_limit_enable = True

        talonfxconfigurator = self.pivotmotor.configurator
        talonfxconfigurator.apply(fx_cfg)

    def periodic(self):
        self.nettable.putNumber("roller_motor_velocity", self.rollermotor.get_velocity().value_as_double)
        self.nettable.putNumber("pivot_motor_velocity", self.pivotmotor.get_velocity().value_as_double)
        self.nettable.putNumber("pivot_setpoint", self.pivotsetpoint)
        self.nettable.putNumber("pivot_duty_cycle", self.pivotmotor.get())
        self.nettable.putNumber("roller_duty_cycle", self.rollermotor.get())
        self.nettable.putNumber("pivot_motor_current", self.pivotmotor.get_torque_current().value_as_double)
        self.nettable.putNumber("roller_motor_current", self.rollermotor.get_stator_current().value_as_double)
        self.nettable.putNumber("pivot_motor_voltage", self.pivotmotor.get_motor_voltage().value_as_double)
        self.nettable.putNumber("roller_motor_voltage", self.rollermotor.get_motor_voltage().value_as_double)
        self.nettable.putNumber("pivot_motor_position", self.pivotmotor.get_position().value_as_double)
        self.nettable.putNumber("roller_motor_position", self.rollermotor.get_position().value_as_double)
        self.nettable.putNumber("pivot_motor_temp", self.pivotmotor.get_device_temp().value_as_double)
        self.nettable.putNumber("roller_motor_temp", self.rollermotor.get_device_temp().value_as_double)

        pivotPIDcalculate=self.pivotPID.calculate(self.get_angle(),degreesToRotations(self.pivotsetpoint.degrees()))
        self.nettable.putNumber("pivotPID_calculate", pivotPIDcalculate)
        self.pivotmotor.set(pivotPIDcalculate)
        
    def simulationPeriodic(self):
        self._armSim.setInputVoltage(self.pivotmotor.get() * 12)
        self._armSim.update(0.02)
        rotation_rotationsPerSecond = radiansToRotations( self._armSim.getVelocity() ) * self._gearRatio
        self.pivotmotor.sim_state.add_rotor_position( rotation_rotationsPerSecond * 0.02 )
        self.pivotmotor.sim_state.set_rotor_velocity( rotation_rotationsPerSecond )

        speed_rotationsPerPeriodic = radiansToRotations( self.simTalon.freeSpeed * self.rollermotor.get() )
        self.rollermotor.sim_state.add_rotor_position( speed_rotationsPerPeriodic * 0.02 )
        self.rollermotor.sim_state.set_rotor_velocity( speed_rotationsPerPeriodic )
        
    def get_speed(self):
        return self.rollermotor.get_velocity().value_as_double
    
    def get_angle(self):
        return self.pivotmotor.get_position().value_as_double

    def set_speed(self, speed):
        speed = max(min(speed,100),0)

    def set_angle(self, angle :degrees):
        angle = max(min(angle,110),0)

        self.pivotsetpoint = Rotation2d.fromDegrees(angle)