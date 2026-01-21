from phoenix6.hardware import TalonFX
from commands2 import Subsystem
from enum import auto
from wpilib import XboxController


class shooterAuto(Subsystem):
    
    def __init__(self):
        self.motor = TalonFX(1)
        self.motor2 = TalonFX(2)
        #insert self.sensor here

    def getSpinnerAngle(self):
        return self.motor2.get_position
    
    def setSpinnerAngle(self, angle):
        self.motor.set(angle)

    def getVelocity(self):
        return self.motor2.get_velocity
    
    def setVelocity(self, velocity):
        self.motor.set(velocity)
        self.motor2.set(velocity)
    
    def getSpinnerSpeed(self):
        return self.motor.get_velocity().value_as_double
    
    def setSpinnerSpeed(self, speed):
        self.motor2.set(speed)
    
    def periodic(self):
        pass
        #log motor and sensor info
        #do subsystem work
        #log visual output