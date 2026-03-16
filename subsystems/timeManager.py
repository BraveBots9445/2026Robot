from copy import deepcopy

from enum import Enum

from .BraveLogger import TimerData, BraveLogger

from wpilib import DriverStation, Timer, reportError


class AutoWinner(Enum):
    Err = 0
    OurAlliance = 1
    TheirAlliance = 2


class TimeManager:
    """
    Class to manage the time on the field and which shift it currently is
    """

    _winner: AutoWinner

    _teleopTimer: Timer

    _shiftTimer: Timer

    _shiftCounter: int = 0

    _data: TimerData

    def __init__(self) -> None:
        self._winner = AutoWinner.Err

        self._teleopTimer = Timer()
        self._shiftTimer = Timer()

        self._data = TimerData(self._winner.value, 0, 0, 0, 0, 0)

    def periodic(self):
        """
        This is not a subsytem periodic, the robot must add this as a periodic.
        """

        if self._winner == AutoWinner.Err:
            msg = DriverStation.getGameSpecificMessage()
            if msg:
                isBlue = DriverStation.getAlliance() == DriverStation.Alliance.kBlue
                if msg == "R":
                    if isBlue:
                        self._winner = AutoWinner.TheirAlliance
                    else:
                        self._winner = AutoWinner.OurAlliance
                elif msg == "B":
                    if isBlue:
                        self._winner = AutoWinner.OurAlliance
                    else:
                        self._winner = AutoWinner.TheirAlliance
                else:
                    reportError(
                        f"Got a bad result from the FMS for winning alliance {msg}"
                    )
                self._data.autoWinner = self._winner.value

        if self._teleopTimer.hasElapsed(10):
            self._shiftTimer.start()

        if self._shiftTimer.advanceIfElapsed(25):
            self._shiftCounter += 1

        self._data.currentShift = self._shiftCounter
        self._data.shiftTime = self._shiftTimer.get()
        self._data.matchTime = self._teleopTimer.get()

        BraveLogger.pushSubsystemData(deepcopy(self._data))

    def startTeleop(self) -> None:
        self._teleopTimer.start()

    def overrideAutoWinner(self, newWinner: AutoWinner) -> None:
        self._winner = newWinner
        self._data.autoWinner = newWinner.value
