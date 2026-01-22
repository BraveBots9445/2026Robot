#Brain_storm_INIT!!!
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

class climber(commands2.Subsystem):

    def __init__(self):
        #def motors
        #def uhh... other stuff
        self.L_Elevator=TalonFX(1)
        self.R_Elevator=TalonFX(2)
        self.L_Climber=TalonFX(3)
        self.R_Climber=TalonFX(4)
        self.elevatorPair=(self.L_Elevator,self.R_Elevator)
        self.climberPair=(self.L_Climber,self.R_Climber)

    def periodic(self):
        pass

    def set_height(self,positionMeWanty):
        
        pass

    def test(self):
        pass

"""
    ||    ||
  <=++====++=>
    ||    ||         <--bad guy ):<
  <=++====++=>
    ||    ||
  <=++====++=>
    ||    ||
   /II    II/
----++----++----"""