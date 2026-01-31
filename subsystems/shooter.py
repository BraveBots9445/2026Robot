from phoenix6.hardware import TalonFX
from commands2 import Subsystem
from enum import auto
from wpilib import XboxController
from wpimath.units import inches
from wpimath.geometry import Translation2d
from wpimath.system.plant import DCMotor
from phoenix6 import controls
from commands import commandTemplate
from wpimath.units import radiansToRotations, meters_per_second
from wpimath.controller import PIDController
from wpimath import controller
from wpilib import SmartDashboard

class Shooter(Subsystem):
    _varName1: TalonFX
    _desiredPosition: inches

    def __init__(self):
        self.motor = TalonFX(1)
        self.motor2 = TalonFX(2)
        #insert self.sensor here
        self.velocity: meters_per_second = 0
        self.angle = 0#%
        self.velocity_voltage = controls.VelocityVoltage(0).with_slot(0)
        self.simTalon = DCMotor.krakenX60(1)
        self.PID = PIDController(Kp=1, Ki=0, Kd=0)

    
    def getSpinnerAngle(self):
        return self.motor2.get_position
    
    def setSpinnerAngle(self, angle):
        self.motor.set(angle)

    def setVelocity(self, velocity: meters_per_second):
        self.velocity = velocity

    def getVelocity(self):
        return self.motor2.get_velocity
    
    def getSpinnerSpeed(self):
        return self.motor.get_velocity().value_as_double
    
    def setSpinnerSpeed(self, speed):
        self.motor2.set(speed)
    
    def periodic(self):
        speed = 0.0 # must be between -1.0 and 1.0
        if self.velocity > 10:
            speed = 0.25
        elif self.velocity > 25:
            speed = 0.50
        elif self.velocity > 50:
            speed = 0.75
        elif self.velocity > 75:
            speed = 1.0
        
        
        

        self.motor.set(speed)
        
        #log motor and sensor info
        #do subsystem work
        #log visual output
    
    def simulationPeriodic(self) -> None:
        simState = self.motor.sim_state
        vel = radiansToRotations(DCMotor.krakenX60(1).freeSpeed * self.motor.get())
        simState.add_rotor_position(vel * 0.02)
        simState.set_rotor_velocity(vel)
    
    def getCurrentPosition(self) -> inches:
        return self._varName1.get_position().value_as_double
    
    def setCurrentPosition(self, position:inches) -> None:
        pass
    
    def getDesiredPosition(self) -> inches:
        return self._desiredPosition
    
    def setDesiredPosition(self) -> None:
        pass
    
    def atPosition(self) -> bool:
        return (self.getCurrentPosition() == self.getDesiredPosition())
    
    PIDController (auto, auto, auto)
    #controller.calculate(current, setPoint)
    #motor.set(PIDcalculation)
    #[Tune]