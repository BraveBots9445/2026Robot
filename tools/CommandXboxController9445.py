from typing import Callable

from wpimath import applyDeadband

from commands2.button import CommandXboxController


class CommandController9445(CommandXboxController):
    """
    An Xbox Controller with specific bindings based on deadbanding, sensitivity curves, and other customizations.
    """

    _controller: CommandXboxController
    """
    The controller object at the specified port
    """

    _deadband: float
    """
    A deadband value to apply to joystick inputs
    This is by percent of full range (0.0 to 1.0)
    0.05 is a common value
    """

    _smoothingFunction: Callable[[float], float]
    """
    A function to apply to joystick inputs for sensitivity adjustment
    A common value is x * abs(x)
    """

    def __init__(
        self,
        port: int,
        deadband: float = 0.05,
        smoothingFunction: Callable[[float], float] = lambda x: x * abs(x),
    ):
        """
        Construct the CommandController9445

        :param port: The DS port the controller is connected to
        :type port: int
        :param deadband: The deadband to apply to joystick inputs as a percent of full range (0.0 to 1.0)
        :type deadband: float
        :param smoothingFunction: A function to apply to joystick inputs for sensitivity adjustment. It should take a [-1.0, 1.0] float and return a [-1.0, 1.0] float.
        :type smoothingFunction: Callable[[float], float]
        """
        super().__init__(port)
        self._deadband = deadband
        self._smoothingFunction = smoothingFunction

    def getFRCLX(self) -> float:
        """
        Get the left joystick X value with deadband and smoothing applied.
        X is positive forwards

        :return: The processed left joystick X value
        :rtype: float
        """
        raw_value = -self.getLeftY()
        raw_value = applyDeadband(raw_value, self._deadband)
        return self._smoothingFunction(raw_value)

    def getFRCLY(self) -> float:
        """
        Get the left joystick Y value with deadband and smoothing applied.

        :return: The processed left joystick Y value
        :rtype: float
        """
        raw_value = -self.getLeftX()
        raw_value = applyDeadband(raw_value, self._deadband)
        return self._smoothingFunction(raw_value)

    def getFRCRX(self) -> float:
        """
        Get the right joystick X value with deadband and smoothing applied.

        :return: The processed right joystick X value
        :rtype: float
        """
        raw_value = -self.getRightY()
        raw_value = applyDeadband(raw_value, self._deadband)
        return self._smoothingFunction(raw_value)

    def getFRCRY(self) -> float:
        """
        Get the right joystick Y value with deadband and smoothing applied.

        :return: The processed right joystick Y value
        :rtype: float
        """
        raw_value = -self.getRightX()
        raw_value = applyDeadband(raw_value, self._deadband)
        return self._smoothingFunction(raw_value)
