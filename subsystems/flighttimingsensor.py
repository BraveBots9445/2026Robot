import wpilib
from playingwithfusion import TimeOfFlight
from commands2 import Subsystem

class FTS(Subsystem):
    def __init__(self):
        # Initialize the ToF sensor on CAN ID 1
        self.tof = TimeOfFlight(1)

    def periodic(self):
        # Get distance in mm
        distance = self.tof.getRange()
        wpilib.SmartDashboard.putNumber("ToF Distance (mm)", distance)