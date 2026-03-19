from commands2.button import CommandGenericHID, Trigger

import wpilib


class ButtonBoard(CommandGenericHID):
    def __init__(self, port: int):
        super().__init__(port)

    def getButton(self, row: int, col: int) -> Trigger:
        """
        Take the 0 indexed row and column of the button board and return a Trigger for that button.
        """
        # if row < 0 or row > 3 or col < 0 or col > 1:
        #     wpilib.reportError(f"Invalid button coordinates: row {row}, col {col}")
        #     return Trigger(lambda: False)
        return self.button(row * 2 + col)

    def getX(self) -> float:
        """
        Get the value of the forward/backward axis of the joystick.
        No deadband is applied because the joystick is digital, so it will only return 0 or 1.
        """
        return self.getRawAxis(0)

    def getY(self) -> float:
        """
        Get the value of the left/right axis of the joystick.
        No deadband is applied because the joystick is digital, so it will only return 0 or 1.
        """
        return self.getRawAxis(1)

    def getForward(self) -> Trigger:
        return Trigger(lambda: self.getX() > 0.5)

    def getReverse(self) -> Trigger:
        return Trigger(lambda: self.getX() < -0.5)
