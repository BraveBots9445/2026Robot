from commands2 import Command
from phoenix6 import swerve
from subsystems.ctredrivetrain import CommandSwerveDrivetrain


class DriveBrake(Command):
    """
    Command to lock the drivetrain in place using SwerveDriveBrake.
    This rotates all swerve modules into an X-pattern to resist being pushed.
    Useful for defense or holding position.
    """

    def __init__(self, drivetrain: CommandSwerveDrivetrain):
        """
        Initialize the DriveBrake command.

        :param drivetrain: The swerve drivetrain subsystem to control
        :type drivetrain: CommandSwerveDrivetrain
        """
        super().__init__()
        self.drivetrain = drivetrain
        
        # Create the brake request that locks wheels in X-pattern
        self.brake_request = swerve.requests.SwerveDriveBrake()
        
        # Declare subsystem dependency
        self.addRequirements(drivetrain)

    def execute(self):
        """
        Execute the command by applying the brake request to the drivetrain.
        
        The SwerveDriveBrake request rotates each swerve module to form an X-pattern,
        making the robot very difficult to push in any direction.
        """
        self.drivetrain.set_control(self.brake_request)

    def end(self, interrupted: bool):
        """
        Called when the command ends.
        
        :param interrupted: Whether the command was interrupted
        :type interrupted: bool
        
        No cleanup needed as the drivetrain will be controlled by another command.
        """
        pass

    def isFinished(self) -> bool:
        """
        Check if the command is finished.
        
        :returns: Always False - this command runs continuously until interrupted
        :rtype: bool
        """
        return False
