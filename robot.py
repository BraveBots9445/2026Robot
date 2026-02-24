from commands2 import Command, CommandScheduler, TimedCommandRobot

from ntcore import NetworkTableInstance

from wpilib import (
    DriverStation,
    RobotBase,
    TimedRobot,
    DataLogManager,
    Mechanism2d,
    SmartDashboard,
    Timer,
)

from phoenix6.signal_logger import SignalLogger

from robotcontainer import RobotContainer


class Robot(TimedCommandRobot):
    m_autonomousCommand: Command
    m_robotContainer: RobotContainer

    # Initialize Robot
    def robotInit(self):
        self.m_robotContainer = RobotContainer()
        DriverStation.startDataLog(DataLogManager.getLog())
        if RobotBase.isSimulation():
            DataLogManager.stop()
        SignalLogger.stop()
        self._nettable = NetworkTableInstance.getDefault().getTable("datatable")
        self._timePub = self._nettable.getDoubleTopic("time").publish()
        self._timer = Timer()
        self._timer.start()
        self.setNetworkTablesFlushEnabled(False)

    def robotPeriodic(self) -> None:
        self._timePub.set(self._timer.get())
        self._timer.restart()
        # wpilib.reportError(f"Got Error from Command Scheduler: {e}", True)

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
        self.m_robotContainer.set_test_bindings()

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
