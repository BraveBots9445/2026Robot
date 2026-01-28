from commands2 import Subsystem


class SubsystemTemplate(Subsystem):
    def __init__(self):
        # Initialize your subsystem here
        pass

    def periodic(self):
        # This method will be called once per scheduler run
        pass

    def getExample(self) -> int:
        # Example method to return some value
        return 42

    def setExample(self, value: int) -> None:
        # Example method to set some value
        pass
