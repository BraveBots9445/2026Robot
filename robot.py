import wpilib
from phoenix6.hardware import TalonFX
from wpilib import DigitalInput
from subsystems.intake import IntakeSubsystem


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):

        # Create intake motor (CAN ID 1)
        intake_motor = TalonFX(1)

        # Create beam break on DIO port 0
        beam_break = DigitalInput(0)

        # Create intake subsystem and pass both in
        self.intake = IntakeSubsystem(intake_motor, beam_break) 

 

class InvertedBeamBreak:
    def __init__(self, channel):
        self.sensor = wpilib.DigitalInput(channel)

    def get(self):
        return not self.sensor.get()

beam_break = InvertedBeamBreak(0) 

beam_break.get()
 
beam_break = wpilib.DigitalInput(0)
self.intake = IntakeSubsystem(intake_motor, beam_break) 
        
       






