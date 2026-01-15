from subsystems.ctredrivetrain import CommandSwerveDrivetrain

# from subsystems.revdrivetrain import Drivetrain as revDrivetrain

from utils.ConsoleLogger import ConsoleLogger

if False and ConsoleLogger.isRunningOnProdBot():
    Drivtetrain = revDrivetrain
else:
    Drivetrain = CommandSwerveDrivetrain
