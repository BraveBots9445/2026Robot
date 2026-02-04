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
import math

class Shooter(Subsystem):
    _varName1: TalonFX
    _desiredPosition: inches
    
    def __init__(self):
        self.shooterMotor = TalonFX(1)
        self.hoodMotor = TalonFX(2)
        #insert self.sensor here
        self.velocity: meters_per_second = 0
        self.angle = 0#%
        self.velocity_voltage = controls.VelocityVoltage(0).with_slot(0)
        self.simTalon = DCMotor.krakenX60(1)
        self.PID = PIDController(Kp=0.0001, Ki=0, Kd=0)
        self.PID2 = PIDController(Kp=0.003, Ki=0, Kd=0)

    
    def getSpinnerAngle(self):
        return self.hoodMotor.get_position
    
    def setSpinnerAngle(self, angle):
        self.shooterMotor.set(angle)

    def setVelocity(self, velocity: meters_per_second):
        self.velocity = velocity

    def getVelocity(self):
        return self.shooterMotor.get_velocity().value_as_double
    
    def getSpinnerSpeed(self):
        return self.shooterMotor.get_velocity().value_as_double
    
    def setSpinnerSpeed(self, speed):
        self.shooterMotor.set(speed)
    
    def periodic(self):
        newSpeed = self.PID.calculate(self.getVelocity(), self.velocity)
        self.shooterMotor.set(newSpeed)

        
        #log motor and sensor info
        #do subsystem work
        #log visual output
    
    def simulationPeriodic(self) -> None:
        simState = self.shooterMotor.sim_state
        vel = radiansToRotations(DCMotor.krakenX60(1).freeSpeed * self.shooterMotor.get())
        simState.add_rotor_position(vel * 0.02)
        simState.set_rotor_velocity(vel)
    
    def getCurrentHoodPosition(self) -> inches:
        return self._varName1.get_position().value_as_double
    
    def setCurrentHoodPosition(self, position:inches) -> None:
        pass
    
    def getDesiredHoodPosition(self) -> inches:
        return self._desiredPosition
    
    def setDesiredHoodPosition(self) -> None:
        pass
    
    def atHoodPosition(self) -> bool:
        return (self.getCurrentPosition() == self.getDesiredPosition())
    
    #PIDController (auto, auto, auto)
    #controller.calculate(current, setPoint)
    #motor.set(PIDcalculation)
    #[Tune]