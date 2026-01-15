from commands2 import Command, CommandScheduler
from wpilib import (
    DriverStation,
    TimedRobot,
    run,
    DataLogManager,
)
import wpilib

from robotcontainer import RobotContainer


class Robot(TimedRobot):
    m_autonomousCommand: Command
    m_robotContainer: RobotContainer

    # Initialize Robot
    def robotInit(self):
        self.m_robotContainer = RobotContainer()
        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

    def robotPeriodic(self) -> None:
        try:
            CommandScheduler.getInstance().run()
        except Exception as e:
            wpilib.reportError(f"Got Error from Command Scheduler: {e}", True)

    def autonomousInit(self):
        self.m_autonomousCommand = self.m_robotContainer.get_auto_command()

        CommandScheduler.getInstance().schedule(self.m_autonomousCommand)

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        if self.m_autonomousCommand:
            self.m_autonomousCommand.cancel()

    # Teleop Robot Functions
    def teleopInit(self):
        if self.m_robotContainer is not None:
            self.m_robotContainer.set_teleop_bindings()

    def teleopPeriodic(self):
        pass

    def teleopExit(self):
        pass

    # Test Robot Functions
    def testInit(self) -> None:
        pass

    def testPeriodic(self):
        pass

    def testExit(self):
        pass

    # Disabled Robot Functions
    def disabledInit(self):
        pass

    def disabledPeriodic(self) -> None:
        pass

    def disabledExit(self):
        pass

    # Simulation Robot Functions
    def _simulationInit(self) -> None:
        pass

    def _simulationPeriodic(self) -> None:
        pass


# Start the Robot when Executing Code
if __name__ == "__main__":
    run(Robot)
