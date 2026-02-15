from commands2 import Command, CommandScheduler
from wpilib import (
    DriverStation,
    TimedRobot,
    run,
    DataLogManager,
)
import wpilib
from ntcore import NetworkTableInstance

from robotcontainer import RobotContainer


from subsystems.WaveshareToF import WaveshareTof, ToFData


class Robot(TimedRobot):
    m_autonomousCommand: Command
    m_robotContainer: RobotContainer

    # Initialize Robot
    def robotInit(self):
        # self.m_robotContainer = RobotContainer()
        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

        self.counter = 31
        self.id = 0
        self.tmpNettable = NetworkTableInstance.getDefault().getTable("0000robot")
        self.dataPub = self.tmpNettable.getStructTopic("tof_data", ToFData).publish()
        self.dataNonePub = self.tmpNettable.getBooleanTopic("tof_dataNone").publish()
        self.tof = WaveshareTof(self.id)

    def robotPeriodic(self) -> None:
        # CommandScheduler.getInstance().run()
        self.counter = (self.counter + 1) % 50
        # if self.counter == 0:
        #     del self.tof
        #     self.id += 1
        #     self.tof = WaveshareTof(self.id)
        #     self.tmpNettable.putNumber("id", self.id)
        #     data = self.tof.getData()
        #     self.dataPub.set(data or ToFData(-1, -1, -1, -1.0))
        #     self.dataNonePub.set(data is None)
        #     if data is not None:
        #         self.tmpNettable.putNumber("Good ID", self.id)
        # wpilib.reportError(f"Got Error from Command Scheduler: {e}", True)
        self.tof._updateMeasurements()
        data = self.tof.getData()
        self.dataPub.set(data or ToFData(-1, -1, -1, -1.0))
        self.dataNonePub.set(data is None)

    def autonomousInit(self):
        return
        self.m_autonomousCommand = self.m_robotContainer.get_auto_command()

        CommandScheduler.getInstance().schedule(self.m_autonomousCommand)

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        if self.m_autonomousCommand:
            self.m_autonomousCommand.cancel()

    # Teleop Robot Functions
    def teleopInit(self):
        return
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
