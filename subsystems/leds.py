import wpilib
from commands2 import Subsystem
kLEDBuffer = 10

class lights(Subsystem):
    def __init__(self):
        self.led = wpilib.AddressableLED(7)
        self.ledData = wpilib.AddressableLED.LEDData() 
        for _ in range(kLEDBuffer):
            self.rainbowFirstPixelHue = 0
            self.led.setLength(kLEDBuffer)
            self.led.setData(self.ledData)
            self.ledData.start()

    def robotPeriodic(self):
        self.rainbow()
        self.led.setData(self.ledData)

    def rainbow(self):
        for i in range(kLEDBuffer):
            hue = (self.rainbowFirstPixelHue + (i * 180 / kLEDBuffer)) % 180
            self.ledData[i].setHSV(int(hue), 255, 128)
        self.rainbowFirstPixelHue += 3
        self.rainbowFirstPixelHue %= 180