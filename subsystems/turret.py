#turret thing ?

from commands2 import Command 
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from phoenix6.hardware import TalonFX
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController

class Turret(Subsystem):
    

    simTalon = DCMotor.krakenX60()

    def __init__(self):
        self.nettable = NetworkTableInstance.getDefault().getTable("000Turret")
        self.rotationmotor = TalonFX(1)
        self.angle = 0
        PIDController(0.1, 7, 10)

    def periodic(self):
        self.nettable.putNumber("setpoint", self.angle)

        # self.rotationmotor.set(self.angle)
        
        distance = abs(self.get_angle() - self.angle)
        speed = 1

        PIDController(0.1, 7, 10)
        pid = PIDController.calculate(self.get_angle(), self.angle)
        self.rotationmotor.set(pid)


        self.nettable.putNumber("speed", speed)

        if self.get_angle() < self.angle:
            self.rotationmotor.set(1 * speed)
        elif self.get_angle() > self.angle:
            self.rotationmotor.set(-1 * speed)
        else:
            self.rotationmotor.set(0)

        # self.nettable.putNumber("motor_current", self.spinmotor.get_torque_current().value_as_double)

    def simulationPeriodic(self):
        rotation_rotationsPerSecond = radiansToRotations( self.simTalon.freeSpeed * self.rotationmotor.get() ) 
        self.rotationmotor.sim_state.add_rotor_position( rotation_rotationsPerSecond * 0.02 )
        self.rotationmotor.sim_state.set_rotor_velocity( rotation_rotationsPerSecond )
    

    def get_angle(self):
        return self.rotationmotor.get_position().value_as_double
    
    def atsetpoint(self):
        if abs(self.get_angle() - self.angle) < 5:
            return True
        else:
            return False


    def set_angle(self, angle):
        r2d = Rotation2d.fromDegrees(angle)
        self.angle = r2d.degrees()
   
    
