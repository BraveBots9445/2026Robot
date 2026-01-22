"""
You do not need to modify this file, but you are welcome to read it.
Note that simulation is not accurately implemented for this subsystem.
You will just look at the hardware tab in the simulation environment to see motor duty cycle activity
"""

from commands2 import Subsystem

from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityDutyCycle
from phoenix6.configs import TalonFXConfiguration, Slot0Configs

from wpilib import DigitalInput
from ntcore import NetworkTableInstance


class MySubsystem(Subsystem):
    def __init__(self):
        # Initialize the network table to be able to log data
        self.nettable = NetworkTableInstance.getDefault().getTable("000MySubsystem")

        # get the motor
        self.motor = TalonFX(10)

        # configure the motor to use simple velocity control
        self.motor.configurator.apply(
            # left as a simple control scheme for practice sake. Can be tuned if desired.
            TalonFXConfiguration().with_slot0(Slot0Configs().with_k_v(0.1)),
        )

        # programmed as NO
        self.limit_switch = DigitalInput(0)

        self.setpoint: float = 0.0  # rotations per second

        self.tolerance: float = 5.0  # rotations per second

    def periodic(self) -> None:
        # log out inputs
        self.nettable.putNumber("setpoint", self.get_setpoint())
        self.nettable.putBoolean("switch_triggered", self.get_switch_triggered())

        # update motor control
        self.motor.set_control(VelocityDutyCycle(self.get_setpoint()))

        # log out outputs
        self.nettable.putNumber("velocity", self.get_velocity())
        self.nettable.putBoolean("at_setpoint", self.at_setpoint())
        self.nettable.putNumber(
            "current", self.motor.get_torque_current().value_as_double
        )

    def set_setpoint(self, setpoint: float) -> None:
        """
        setpoint: float - rotations per second
        """
        self.setpoint = setpoint

    def get_setpoint(self) -> float:
        """
        returns float - rotations per second
        """
        return self.setpoint

    def get_velocity(self) -> float:
        """
        returns float - rotations per second
        """
        return self.motor.get_velocity().value_as_double

    def get_switch_triggered(self) -> bool:
        """
        returns bool - True if the limit switch is pressed, False otherwise
        """
        return self.limit_switch.get()

    def at_setpoint(self) -> bool:
        """
        returns bool - True if the motor velocity is within the tolerance of the setpoint
        """
        return abs(self.get_velocity() - self.get_setpoint()) < self.tolerance
