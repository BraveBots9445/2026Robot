from commands2 import Command 
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from phoenix6.hardware import TalonFX
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations
from wpimath.controller import PIDController
from wpilib import SmartDashboard

class Intake(Subsystem):
    simTalon = DCMotor.krakenX60()

    def __init__(self,):
        self.nettable = NetworkTableInstance.getDefault().getTable("LogInputs")
        self.subtable = NetworkTableInstance.getDefault().getTable("000Intake")
        self.rotationmotor = TalonFX(1)
        self.spinmotor = TalonFX(2)
        self.setpoint = 0
        self.speed = 0
        self.angle = 0
        self.PID = PIDController(Kp=0.003, Ki=0, Kd=0)
        SmartDashboard.putData(self.PID)

    def periodic(self):
        self.nettable.putNumber("spin/velocity", self.spinmotor.get_velocity().value_as_double)
        self.nettable.putNumber("spin/dutycycle", self.spinmotor.get())
        self.nettable.putNumber("spin/current", self.spinmotor.get_stator_current().value_as_double)
        self.nettable.putNumber("spin/voltage", self.spinmotor.get_motor_voltage().value_as_double)
        self.nettable.putNumber("spin/position", self.spinmotor.get_position().value_as_double)
        self.nettable.putNumber("spin/temp", self.spinmotor.get_device_temp().value_as_double)
        self.nettable.putNumber("rotation/velocity", self.rotationmotor.get_velocity().value_as_double)
        self.nettable.putNumber("rotation/dutycycle", self.rotationmotor.get())
        self.nettable.putNumber("rotation/current", self.rotationmotor.get_torque_current().value_as_double)
        self.nettable.putNumber("rotation/voltage", self.rotationmotor.get_motor_voltage().value_as_double)
        self.nettable.putNumber("rotation/position", self.rotationmotor.get_position().value_as_double)
        self.nettable.putNumber("rotation/temp", self.rotationmotor.get_device_temp().value_as_double)
        self.nettable.putNumber("spin/temp", self.spinmotor.get_device_temp().value_as_double)

        self.subtable.putNumber("set_point", self.setpoint)

        self.PIDcalcuate=self.PID.calculate(self.get_speed(),self.setpoint)
        self.subtable.putNumber("PID_calculate", self.PIDcalcuate)
        self.rotationmotor.set(self.angle)

        self.spinmotor.set(self.spinmotor.get() + self.PIDcalcuate)
        
    def simulationPeriodic(self):
        rotation_rotationsPerSecond = radiansToRotations( self.simTalon.freeSpeed * self.rotationmotor.get() ) 
        self.rotationmotor.sim_state.add_rotor_position( rotation_rotationsPerSecond * 0.02 )
        self.rotationmotor.sim_state.set_rotor_velocity( rotation_rotationsPerSecond )

        speed_rotationsPerPeriodic = radiansToRotations( self.simTalon.freeSpeed * self.spinmotor.get() )
        self.spinmotor.sim_state.add_rotor_position( speed_rotationsPerPeriodic * 0.02 )
        self.spinmotor.sim_state.set_rotor_velocity( speed_rotationsPerPeriodic )
        
    def get_speed(self):
        return self.spinmotor.get_velocity().value_as_double
    
    def get_angle(self):
        return self.rotationmotor.get_position().value_as_double

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
    
    def setsetpoint(self, setpoint):
        self.setpoint = setpoint