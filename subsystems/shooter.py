from phoenix6.hardware import TalonFX
from commands2 import Subsystem
from enum import auto
from wpilib import XboxController
from wpimath.units import inches
from wpimath.geometry import Translation2d
from phoenix6 import controls
from commands import commandTemplate

class Shooter(Subsystem):
    _varName1: TalonFX
    _desiredPosition: inches

    def __init__(self):
        self.motor = TalonFX(1)
        self.motor2 = TalonFX(2)
        #insert self.sensor here
        self.velocity = 0
        self.angle = 0#%
        self.velocity_voltage = controls.VelocityVoltage(0).with_slot(0)

    def getSpinnerAngle(self):
        return self.motor2.get_position
    
    def setSpinnerAngle(self, angle):
        self.motor.set(angle)

    def setVelocity(self, velocity):
        self.velocity = velocity

    def getVelocity(self):
        return self.motor2.get_velocity
    
    def getSpinnerSpeed(self):
        return self.motor.get_velocity().value_as_double
    
    def setSpinnerSpeed(self, speed):
        self.motor2.set(speed)
    
    def periodic(self):
        self.motor.set(self.velocity)
        #log motor and sensor info
        #do subsystem work
        #log visual output
    
    def simulationPeriodic(self) -> None:
        pass
    
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
    