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
from wpimath.units import radiansToRotations

class Climber(commands2.Subsystem):

  def __init__(self):
    #def motors
    #def uhh... other stuff
    self.LElevator=TalonFX(1)
    self.RElevator=TalonFX(2)
    self.simTalon=DCMotor.krakenX60(1)
    self.LWanty=0
    self.RWanty=0
    self.velocity=1

  def periodic(self):
#    Lheight=self.getCurrentPosition("L")*3.14*3
#    if Lheight>self.getDesiredPosition("L"):
    self.LElevator.set(self.velocity)
  
  def simulationPeriodic(self) -> None:
    rotation_rotationsPerSecond = radiansToRotations( self.simTalon.freeSpeed * self.LElevator.get() )
    self.LElevator.sim_state.add_rotor_position ( rotation_rotationsPerSecond * 0.02 )
    self.LElevator.sim_state.set_rotor_velocity ( rotation_rotationsPerSecond )

    rotation_rotationsPerSecond = radiansToRotations( self.simTalon.freeSpeed * self.RElevator.get() )
    self.RElevator.sim_state.add_rotor_position ( rotation_rotationsPerSecond * 0.02)
    self.RElevator.sim_state.set_rotor_velocity ( rotation_rotationsPerSecond )
  
  def getCurrentPosition(self,side) -> inches:
    if side==("L"):
      return self.LElevator.get_position().value_as_double
    elif side==("R"):
      return self.RElevator.get_position().value_as_double
    else:
      self.error

  def setCurrentPosition(self) -> None:
    pass
  
  def getDesiredPosition(self,side) -> inches:
    if side==("L"):
      return self.LWanty
    elif side==("R"):
      return self.RWanty
    else:
      self.error
  
  def setDesiredPosition(self,position,side) -> None:
    if side==("L"):
      self.LWanty=(position)
    elif side==("R"):
      self.RWanty=(position)
    else:
      self.error

  def atPosition(self) -> bool:
    return ( self.getCurrentPosition() == self.getDesiredPosition() )
  
  def setVelocity(self,V):
    self.velocity = V
    
  def getHeight(self):
    return

  def error(self):
    print ("I don't know what side that's pertaining to! use L or R when specifying which elevator.")

"""
    ||    ||
  <=++====++=>
    ||    ||         <--bad guy ]:<
  <=++====++=>
    ||    ||
  <=++====++=>
    ||    ||
   /II    II/    [_]
----++----++----"""