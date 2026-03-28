from commands2 import Command
from phoenix6 import swerve
from subsystems.ctredrivetrain import CommandSwerveDrivetrain
from wpimath.geometry import Rotation2d


class DriveReset(Command):
    """
    Command to point all swerve wheels in a specific direction.
    Useful for resetting modules to a known orientation (typically forward).
    """

    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        direction: Rotation2d = Rotation2d.fromDegrees(0),
    ):
        """
        Initialize the DriveReset command.

        :param drivetrain: The swerve drivetrain subsystem to control
        :type drivetrain: CommandSwerveDrivetrain
        :param direction: The direction to point all wheels (default: 0° forward)
        :type direction: Rotation2d
        """
        super().__init__()
        self.drivetrain = drivetrain
        
        # Create the point request to align all wheels to the specified direction
        self.point_request = swerve.requests.PointWheelsAt().with_module_direction(
            direction
        )
        
        # Declare subsystem dependency
        self.addRequirements(drivetrain)

    def execute(self):
        """
        Execute the command by applying the point request to the drivetrain.
        
        The PointWheelsAt request rotates each swerve module to point in the
        specified direction, typically used to reset modules to forward (0°).
        """
        self.drivetrain.set_control(self.point_request)

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