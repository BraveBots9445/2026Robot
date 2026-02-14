from commands2 import Subsystem
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations, degrees
from wpimath.controller import PIDController
from wpilib import SmartDashboard
from ntcore import NetworkTableInstance

from phoenix6.hardware import TalonFX,CANcoder

class Intake(Subsystem):
    simTalon = DCMotor.krakenX60()

    def __init__(self,):
        self.nettable = NetworkTableInstance.getDefault().getTable("000Intake")
        self.pivotmotor = TalonFX(20)
        self.rollermotor = TalonFX(21)
        self.rollersetpoint = 0
        self.pivotsetpoint = 0
        self.speed = 0
        self.angle = 0
        self.encoder = CANcoder(20)
        self.rollerPID = PIDController(Kp=0.003, Ki=0, Kd=0)
        SmartDashboard.putData(self.rollerPID)
        self.pivotPID = PIDController(Kp=0.131, Ki=0, Kd=0)
        SmartDashboard.putData(self.pivotPID)

    def periodic(self):
        self.nettable.putNumber("roller_motor_velocity", self.rollermotor.get_velocity().value_as_double)
        self.nettable.putNumber("pivot_motor_velocity", self.pivotmotor.get_velocity().value_as_double)
        self.nettable.putNumber("roller_setpoint", self.rollersetpoint)
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

        self.rollerPIDcalcuate=self.rollerPID.calculate(self.get_speed(),self.rollersetpoint)
        self.pivotPIDcalcuate=self.pivotPID.calculate(self.get_angle(),self.pivotsetpoint)
        self.nettable.putNumber("rollerPID_calculate", self.rollerPIDcalcuate)
        self.nettable.putNumber("pivotPID_calculate", self.pivotPIDcalcuate)
        self.pivotmotor.set(self.pivotPIDcalcuate)

        self.rollermotor.set(self.rollerPIDcalcuate)
        self.pivotmotor.set(self.pivotPIDcalcuate)
        
    def simulationPeriodic(self):
        rotation_rotationsPerSecond = radiansToRotations( self.simTalon.freeSpeed * self.pivotmotor.get() ) 
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
        if speed > 1:
            speed = 1
        if speed < -1:
            speed = -1
        self.speed = speed
    def set_angle(self, angle):
        if angle > 1:
            angle = 1
        if angle < -1:
            angle = -1
        self.angle = angle
    
    def setrollersetpoint(self, rollersetpoint):
        self.rollersetpoint = rollersetpoint
    
    def setpivotsetpoint(self, pivotsetpoint: degrees):
        self.pivotsetpoint = pivotsetpoint