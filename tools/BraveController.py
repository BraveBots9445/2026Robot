from commands2.button import CommandXboxController
from wpimath import applyDeadband


class BraveController(CommandXboxController):
    """
    Enhanced Xbox controller with built-in deadband and sqrt input support.
    Axes are rotated 90° clockwise to match FRC coordinate system (X forward, Y right).
    """

    def __init__(self, port: int, deadband: float = 0.05, sqrt_inputs: bool = True):
        """
        Initialize the BraveController.

        Args:
            port: The port number for the controller.
            deadband: The deadband value to apply to joystick inputs. Defaults to 0.05.
            sqrt_inputs: Whether to apply sqrt scaling to inputs for finer control. Defaults to True.
        """
        super().__init__(port)
        self.deadband = deadband
        self.sqrt_inputs = sqrt_inputs

    def getLeftY(self) -> float:
        """Get left Y axis (left is positive) with deadband and optional sqrt scaling."""
        value = -applyDeadband(super().getLeftX(), self.deadband)
        if self.sqrt_inputs:
            return value * abs(value)
        return value

    def getLeftX(self) -> float:
        """Get left X axis (forward is positive) with deadband and optional sqrt scaling."""
        value = -applyDeadband(super().getLeftY(), self.deadband)
        if self.sqrt_inputs:
            return value * abs(value)
        return value

    def getRightY(self) -> float:
        """Get right Y axis (left is positive) with deadband and optional sqrt scaling."""
        value = -applyDeadband(super().getRightX(), self.deadband)
        if self.sqrt_inputs:
            return value * abs(value)
        return value

    def getRightX(self) -> float:
        """Get right X axis (forward is positive) with deadband and optional sqrt scaling."""
        value = -applyDeadband(super().getRightY(), self.deadband)
        if self.sqrt_inputs:
            return value * abs(value)
        return value
