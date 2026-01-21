from commands2.button import CommandXboxController
from commands2 import Command, cmd
import commands2
from commands.intake import IntakeCommand
from commands.score import Score
from commands.score_slow import ScoreSlow
from subsystems.my_subsystem import MySubsystem
from subsystems.leds import LEDSubsystem
from shootercode import shooterAuto

class RobotContainer:
    def __init__(self):
        self.leds = LEDSubsystem()
        self.joystick = CommandXboxController(0)
        self.intake = MySubsystem()
        self.shooterAuto = shooterAuto()

    def set_teleop_bindings(self):
        # while the left trigger is held, run the intake command
        self.joystick.leftTrigger().whileTrue(IntakeCommand(self.intake))
        # while the right trigger is held, run the score command
        self.joystick.rightTrigger().whileTrue(Score(self.intake))
        # while the right bumper is held, run the score slow command
        self.joystick.rightBumper().whileTrue(ScoreSlow(self.intake))
        self.joystick.b().onTrue( cmd.runOnce(lambda: self.shooterAuto.setSpinnerAngle(75)))
        self.joystick.b().onFalse( cmd.runOnce(lambda: self.shooterAuto.setSpinnerAngle(0)))
        self.joystick.y().onTrue( cmd.runOnce(lambda: self.shooterAuto.setSpinnerSpeed(30)))
        self.joystick.y().onFalse( cmd.runOnce(lambda: self.shooterAuto.setSpinnerSpeed(0)))

    """
    BONUS: Implement this method to return the autonomous command.

    Your autonomous command should do the following:
        1. Run the intake command for 3 seconds or until the limit switch is triggered.
        2. Then run the score command until it ends.
    """

    def get_autonomous_command(self) -> Command:
        return commands2.cmd.none()
