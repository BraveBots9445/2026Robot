from phoenix6.hardware import TalonFX

from commands2 import Subsystem

from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations
from wpimath.controller import PIDController

from wpilib import SmartDashboard

from ntcore import NetworkTableInstance 

class Indexer (Subsystem):
    def __init__(self):
        super().__init__()
        self.nettable = NetworkTableInstance.getDefault().getTable("00LogInputs/Indexer")
        self.subtable = NetworkTableInstance.getDefault().getTable("000Indexer")
        self.indexer_motor = TalonFX(25) 
        self.setpoint = 0
        self.simTalon = DCMotor.krakenX60(1) 
        self.PID = PIDController(Kp=0.003, Ki=0, Kd=0)
        SmartDashboard.putData(self.PID)

    def periodic(self):
        self.nettable.putNumber("spin/velocity", self.indexer_motor.get_velocity().value_as_double)
        self.nettable.putNumber("spin/dutycycle", self.indexer_motor.get())
        self.nettable.putNumber("spin/current", self.indexer_motor.get_stator_current().value_as_double)
        self.nettable.putNumber("spin/voltage", self.indexer_motor.get_motor_voltage().value_as_double)
        self.nettable.putNumber("spin/position", self.indexer_motor.get_position().value_as_double)
        self.nettable.putNumber("spin/temp", self.indexer_motor.get_device_temp().value_as_double)
        self.subtable.putNumber("set_point", self.setpoint)

        self.PIDcalcuate=self.PID.calculate(self.getMotorSpeed(),self.setpoint)
        self.subtable.putNumber("PID_calculate", self.PIDcalcuate)
        self.indexer_motor.set(self.indexer_motor.get() + self.PIDcalcuate)
 
    def simulationPeriodic(self):
        pass

    def getMotorSpeed(self):
        return self.indexer_motor.get()
    
    def setMotorSpeed(self, speed):
        self.setpoint = speed 


class Woahval (Subsystem):
    def __init__(self):
        super().__init__()
        self.nettable = NetworkTableInstance.getDefault().getTable("00LogInputs/Woahval")
        self.subtable = NetworkTableInstance.getDefault().getTable("000Woahval")
        self.woahval_motor = TalonFX(8) 
        self.setpoint = 0
        self.simTalon = DCMotor.krakenX60(2) 
        self.PID = PIDController(Kp=0.003, Ki=0, Kd=0)
        SmartDashboard.putData(self.PID)

    def periodic(self):
        self.nettable.putNumber("spin/velocity", self.woahval_motor.get_velocity().value_as_double)
        self.nettable.putNumber("spin/dutycycle", self.woahval_motor.get())
        self.nettable.putNumber("spin/current", self.woahval_motor.get_stator_current().value_as_double)
        self.nettable.putNumber("spin/voltage", self.woahval_motor.get_motor_voltage().value_as_double)
        self.nettable.putNumber("spin/position", self.woahval_motor.get_position().value_as_double)
        self.nettable.putNumber("spin/temp", self.woahval_motor.get_device_temp().value_as_double)
        self.subtable.putNumber("set_point", self.setpoint)

        self.PIDcalcuate=self.PID.calculate(self.getMotorSpeed(),self.setpoint)
        self.subtable.putNumber("PID_calculate", self.PIDcalcuate)
        self.woahval_motor.set(self.woahval_motor.get() + self.PIDcalcuate)
 
    def simulationPeriodic(self):
        pass

    def getMotorSpeed(self):
        return self.woahval_motor.get()
    
    def setMotorSpeed(self, speed):
        self.setpoint = speed 
    
   
    #def run_indexer(self, speed: float):
        #Add logic here to prevent movement if specific conditions are met (e.g., if full)
     #   self.indexer_motor.set(speed)

    #def stop_indexer(self):
      #  self.indexer_motor.set(0)

    #def is_loaded(self):
       # return self.limit_switch_loaded.get()
    
    #def run_forward(self, speed: float):
        #Runs the indexer motor forward
     #   self.indexer_motor.set(speed) 

    #def run_backwards(self, speed: float):
        #Runs the indexer motor backwards
     #   self.indexer_motor.set(speed)                         
     