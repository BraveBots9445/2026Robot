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
        self.leftElevator=TalonFX(1)
        self.rightElevator=TalonFX(2)
        self.leftClimber=TalonFX(3)
        self.rightClimber=TalonFX(4)
        self.elevatorPair=(self.leftElevator,self.rightElevator)
        self.climberPair=(self.leftClimber,self.rightElevator)

    def autoInit(self):
        #get ready for auto
        #double check everything works
        pass

    def autoPeriodic(self):
        #do auto
        #auto upkeep
        pass

    def teleopInit(self):
        #stop auto
        #start getting down from l1
        pass

    def teleopPeriodic(self):
        #do what driver tells me
        #teleop upkeep
        #comfort features
        pass

    def periodic(self):
        pass

    def set(self,positionMeWanty):
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