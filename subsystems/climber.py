#   ***subsystem construction zone***   # 
"""
I need to know:
1. motor velocity
2. actual movement of climber
3. height off ground/location on rungs
4. left/center/right of rung
what it will do:
1. get bot to level 1 during auto
2. respond to driver to climb
3. possibly flatten to go under ravine
4. climb down
"""

import commands2
from phoenix6.hardware import TalonFX
from wpimath.units import inches
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations, inchesToMeters, metersToInches
from wpimath.controller import PIDController
from wpilib import SmartDashboard
import math
from wpilib.simulation import ElevatorSim

class Climber(commands2.Subsystem):

  drumRadius = inchesToMeters( 1 )
  gearing = 2.0

  def __init__(self):
    self.Elevator=TalonFX(1)
    self.simTalon=DCMotor.krakenX60(1)
    self.Wanty=0
    self.velocity=1
    self.setPoint:inches = 0
    self.PID = PIDController(Kp=1.0, Ki=0, Kd=0)
    SmartDashboard.putData(self.PID)
    self.elevSim = ElevatorSim(
      DCMotor.krakenX60(1), #gearbox: DCMotor,
      self.gearing, #gearing: SupportsFloat,
      1.0, #carriageMass: kilograms,
      self.drumRadius, #drumRadius: meters,
      inchesToMeters( 0 ), #minHeight: meters,
      inchesToMeters( 720.0 ), #maxHeight: meters,
      False, #simulateGravity: bool,
      0.0, #startingHeight: meters
    )

  def periodic(self):
    self.PIDcalculate=self.PID.calculate(self.getHeight(), self.setPoint)
    self.Elevator.set( self.PIDcalculate )
    #SmartDashboard.putNumber( "PIDCalculate", self.PIDcalculate )
    SmartDashboard.putNumber( "Setpoint", self.setPoint )
  
  def simulationPeriodic(self) -> None:
    # rotation_rotationsPerSecond = radiansToRotations( self.simTalon.freeSpeed * self.Elevator.get() )
    # self.Elevator.sim_state.add_rotor_position ( rotation_rotationsPerSecond * 0.02 )
    # self.Elevator.sim_state.set_rotor_velocity ( rotation_rotationsPerSecond )

    self.elevSim.update( 0.02 )
    velocity_mps = self.elevSim.getVelocity()  
    velocity_rps = velocity_mps / ( 2 * math.pi * self.drumRadius * self.gearing )
    self.Elevator.sim_state.set_rotor_velocity( velocity_rps )
    self.Elevator.sim_state.add_rotor_position( velocity_rps * 0.02 )
    self.elevSim.setInputVoltage( self.Elevator.get() )

    SmartDashboard.putNumber( "ElevatorSimVelocity", self.elevSim.getVelocity() )
    SmartDashboard.putNumber( "ElevatorSimPosition", self.elevSim.getPositionInches() )
    SmartDashboard.putNumber( "Height", self.getHeight() )

  
  def getCurrentPosition(self) -> inches:
    return self.Elevator.get_position().value_as_double
  

  def setCurrentPosition(self) -> None:
    pass
  
  def getDesiredPosition(self) -> inches:
    return self.Wanty
  
  def setDesiredPosition(self,position) -> None:
    self.Wanty=(position)

  def atPosition(self) -> bool:
    return ( self.getCurrentPosition() == self.getDesiredPosition() )
  
  def setVelocity(self,V):
    self.velocity = V
    
  def getHeight(self) -> inches:
    return metersToInches( self.Elevator.get_rotor_position().value_as_double * 2 * math.pi * self.drumRadius * self.gearing )
  
  def setHeight(self, height: inches): 
    self.setPoint = height

"""
    ||    ||
  <=++====++=>
    ||    ||         <--bad guy ]:<
  <=++====++=>
    ||    ||
  <=++====++=>
    ||    ||
   /II    II/
----++----++----"""