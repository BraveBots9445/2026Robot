from commands2 import Command
import wpilib
from subsystems.leds import Leds

class LedWhite(Command):
    
    def __init__(self, led: Leds) -> None:
        super().__init__()
        self.led = led
        self.addRequirements(led)
       

    def initialize(self) -> None:
        
        pass

    def execute(self):
        self.led.setSolidColor( wpilib.Color.kWhite)
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):    
        pass
    
    def runsWhenDisabled(self):
        return True