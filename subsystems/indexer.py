import wpilib
import wpilib.drive 
from phoenix6.hardware import TalonFX
from commands2 import Subsystem
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations
from wpimath.controller import PIDController

class Indexer (Subsystem):
    def __init__(self):
        super().__init__()
        
        #Put motors and sensors here
        self.indexer_motor = TalonFX(25) 
       # self.top_sensor = wpilib.DigitalInput(top_sensor_port) # type: ignore
        #self.bottom_sensor = wpilib.DigitalInput(bottom_sensor_port)  # type: ignore
        #Limit Switch goes here but i dont know how to write it 
        self.simTalon = DCMotor.krakenX60(1) 
        PIDController (0.5, 0, 0)

    def periodic(self):
        PIDController.calculate (current, setpoint)
        motor.set (PIDcalculation)
        # log sensors
        # proform calcunations 
        # make chages to system (run motors(something with sensors?))
        # log calculations / diagrams / predictions (or something similar with an idexer)
        pass
 
    def simulationPeriodic(self):
        pass

    def getMotorSpeed(self):
        return self.indexer_motor.get()
    
    def setMotorSpeed(self, speed):
        self.indexer_motor.set(speed) 
        

        
    
    





           # old code. here incase I need something from it (probably wont though)
   


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