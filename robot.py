from commands2 import Command, CommandScheduler
from wpilib import DriverStation, TimedRobot, DataLogManager, RobotBase
import wpilib

from phoenix6.signal_logger import SignalLogger

from robotcontainer import RobotContainer


class Robot(TimedRobot):
    m_autonomousCommand: Command
    m_robotContainer: RobotContainer

    # Initialize Robot
    def robotInit(self):
        DriverStation.silenceJoystickConnectionWarning(True)
        if RobotBase.isReal():
            DataLogManager.start()
            DriverStation.startDataLog(DataLogManager.getLog())
        else:
            SignalLogger.set_path("./.logs/sim")
            SignalLogger.stop()
        self.m_robotContainer = RobotContainer()

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()
        # wpilib.reportError(f"Got Error from Command Scheduler: {e}", True)

    def autonomousInit(self):
        # if hasattr(self.m_robotContainer, "shooter"):
        #     self.m_robotContainer.shooter.setHoodIdle(False)
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
            # if hasattr(self.m_robotContainer, "shooter"):
            #     self.m_robotContainer.shooter.setHoodIdle(False)
            self.m_robotContainer.set_teleop_bindings()

    def teleopPeriodic(self):
        pass

    def teleopExit(self):
        pass

    # Test Robot Functions
    def testInit(self) -> None:
        self.m_robotContainer.set_test_bindings()

    def testPeriodic(self):
        pass

    def testExit(self):
        pass

    # Disabled Robot Functions
    def disabledInit(self):
        # if hasattr(self.m_robotContainer, "shooter"):
        #     self.m_robotContainer.shooter.setHoodIdle(False)
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
