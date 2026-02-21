from typing import Callable
from commands2 import Command
from phoenix6 import swerve
from subsystems.ctredrivetrain import CommandSwerveDrivetrain


class DriveByStick(Command):
    """
    Command to drive the robot using joystick inputs.
    Supports both field-centric and robot-centric control modes.
    All speed scaling is handled within this command's execute() method.
    """

    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        velocity_x_supplier: Callable[[], float] = lambda: 0.0,
        velocity_y_supplier: Callable[[], float] = lambda: 0.0,
        rotation_supplier: Callable[[], float] = lambda: 0.0,
        field_centric: bool = True,
    ):
        """
        Initialize the DriveByStick command.

        :param drivetrain: The swerve drivetrain subsystem to control
        :type drivetrain: CommandSwerveDrivetrain
        :param velocity_x_supplier: Supplier returning normalized X velocity (-1.0 to 1.0, forward/backward)
        :type velocity_x_supplier: Callable[[], float]
        :param velocity_y_supplier: Supplier returning normalized Y velocity (-1.0 to 1.0, left/right)
        :type velocity_y_supplier: Callable[[], float]
        :param rotation_supplier: Supplier returning normalized rotation rate (-1.0 to 1.0)
        :type rotation_supplier: Callable[[], float]
        :param field_centric: If True, use field-centric control; if False, use robot-centric
        :type field_centric: bool
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.velocity_x_supplier = velocity_x_supplier
        self.velocity_y_supplier = velocity_y_supplier
        self.rotation_supplier = rotation_supplier
        self.field_centric = field_centric

        # Configure the drive request based on field_centric parameter
        if field_centric:
            self.drive_request = swerve.requests.FieldCentric()
        else:
            self.drive_request = swerve.requests.RobotCentric()

        self.drive_request = (
            self.drive_request.with_deadband(0)  # Deadband handled by BraveController
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

        # Declare subsystem dependency
        self.addRequirements(drivetrain)

    def execute(self):
        """
        Execute the command by applying joystick inputs to the drivetrain.
        
        This method handles ALL speed scaling:
        1. Gets current allowed speeds from drivetrain (respects speed limiting)
        2. Scales normalized controller inputs (-1.0 to 1.0) by allowed speeds
        3. Applies the scaled velocities to the swerve drive request
        """
        # Get allowed speeds from drivetrain (respects speed limiting)
        maxLinear, maxAngular = self.drivetrain.get_allowed_speeds()
        
        # Scale normalized controller inputs by allowed speeds to get actual velocities
        x = self.velocity_x_supplier() * maxLinear
        y = self.velocity_y_supplier() * maxLinear
        rot = self.rotation_supplier() * maxAngular
        
        # Apply the scaled velocities to the drivetrain
        self.drivetrain.set_control(
            self.drive_request.with_velocity_x(x)
            .with_velocity_y(y)
            .with_rotational_rate(rot)
        )

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
