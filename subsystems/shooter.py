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
from rev import SparkMax
from ntcore import NetworkTableInstance
from ntcore import NetworkTable
from wpilib import RobotController



class Shooter(Subsystem):
    _desiredPosition: inches
    
    def __init__(self):
        self.shooterMotor = TalonFX(1)
        self.hoodMotor = SparkMax(1, SparkMax.MotorType.kBrushless)
        self.hoodEncoder = self.hoodMotor.getEncoder()
        #insert self.sensor here
        self.velocity: meters_per_second = 0
        self.angle = 0#%
        self.velocity_voltage = controls.VelocityVoltage(0).with_slot(1)
        self.simTalon = DCMotor.krakenX60(1)
        self.PID = PIDController(Kp=0.0001, Ki=0, Kd=0)
        self.PID2 = PIDController(Kp=0.003, Ki=0, Kd=0)
        self.netTable = NetworkTableInstance.getDefault().getTable("000Shooter")

    def get_position(self):
        return self.hoodEncoder.getPosition()
    
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

        self.netTable.putNumber("flywheel/", self.shooterMotor.get())
        #log motor and sensor info
        #do subsystem work
        #log visual output
    
    def simulationPeriodic(self) -> None:
        simState = self.shooterMotor.sim_state
        vel = radiansToRotations(DCMotor.krakenX60(1).freeSpeed * self.shooterMotor.get())
        simState.add_rotor_position(vel * 0.02)
        simState.set_rotor_velocity(vel)
    
    def getCurrentHoodPosition(self) -> inches:
        return self.hoodMotor
    
    def setCurrentHoodPosition(self, position:inches) -> None:
        pass
    
    def getDesiredHoodPosition(self) -> inches:
        return self._desiredPosition
    
    def setDesiredHoodPosition(self) -> None:
        pass
    
    def atHoodPosition(self) -> bool:
        return (self.getCurrentHoodPosition() == self.getDesiredHoodPosition())
    
    #PIDController (auto, auto, auto)
    #PIDController.calculate#(current, setPoint)
    #shooterMotor:(PIDController.calculate)
    #[Tune]