from commands2 import Command 
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from phoenix6.hardware import TalonFX
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations

class Intake(Subsystem):
    intakenumber = 0
    indexed = 0
    intakelocation = 0
    stored = 0

    simTalon = DCMotor.krakenX60()

    def __init__(self):
        self.nettable = NetworkTableInstance.getDefault().getTable("000Intake")
        self.rotationmotor = TalonFX(1)
        self.spinmotor = TalonFX(2)
        self.speed = 0
        self.angle = 0

    def periodic(self):
        self.nettable.putNumber("motor_speed", self.get_speed())

        self.spinmotor.set(self.speed)
        self.rotationmotor.set(self.angle)

        self.nettable.putNumber("motor_current", self.spinmotor.get_torque_current().value_as_double)

    def simulationPeriodic(self):
        rotation_rotationsPerSecond = radiansToRotations( self.simTalon.freeSpeed * self.rotationmotor.get() ) 
        self.rotationmotor.sim_state.add_rotor_position( rotation_rotationsPerSecond * 0.02 )
        self.rotationmotor.sim_state.set_rotor_velocity( rotation_rotationsPerSecond )

        speed_rotationsPerPeriodic = radiansToRotations( self.simTalon.freeSpeed * self.spinmotor.get() )
        self.spinmotor.sim_state.add_rotor_position( speed_rotationsPerPeriodic * 0.02 )
        self.spinmotor.sim_state.set_rotor_velocity( speed_rotationsPerPeriodic )
        
    def get_speed(self):
        return self.spinmotor.get()
    
    def get_angle(self):
        return self.rotationmotor.get

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