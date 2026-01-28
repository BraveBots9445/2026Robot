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

class ClimberIsCool(commands2.Subsystem):

  def __init__(self,side):
    #def motors
    #def uhh... other stuff
    self.Elevator=TalonFX(side)
    self.positionMeWanty=0

  def periodic(self):
    pass
  
  def simulationPeriodic(self) -> None:
    pass
  
  def getCurrentPosition(self) -> inches:
    return self.Elevator.get_position().value_as_double

  def setCurrentPosition(self) -> None:
    pass
  
  def getDesiredPosition(self) -> inches:
    return self.positionMeWanty
  
  def setDesiredPosition(self) -> None:
    pass
  
  def atPosition(self) -> bool:
    return ( self.getCurrentPosition() == self.getDesiredPosition() )

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