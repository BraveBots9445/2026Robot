from copy import deepcopy

from enum import Enum

from wpimath.units import seconds

from wpilib import DriverStation, Timer, reportError

from .BraveLogger import TimerData, BraveLogger


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

    _earlyTime: seconds = 3

    def __init__(self) -> None:
        self._winner = AutoWinner.Err

        self._teleopTimer = Timer()
        self._shiftTimer = Timer()

        self._data = TimerData(self._winner.value, 0, 0, 0, 0, 0, False, False)

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
            if self._shiftCounter == 0:
                self._shiftCounter += 1

        if self._shiftTimer.advanceIfElapsed(25):
            self._shiftCounter += 1

        self._data.ourActivePeriod = self._isHubActive()
        self._data.rawOurActivePeriod = (
            self._shiftCounter == 0
            or self._shiftCounter % 2
            == (0 if self._winner == AutoWinner.OurAlliance else 1)
            or self._shiftCounter > 4
        )
        self._data.matchTimeUp = self._teleopTimer.get()
        self._data.matchTimeDown = 140 - self._data.matchTimeUp
        self._data.currentShift = self._shiftCounter
        self._data.shiftTime = self._shiftTimer.get()
        if not self._teleopTimer.hasElapsed(10):
            self._data.timeLeftInShift = 10 - self._teleopTimer.get()
        elif self._data.ourActivePeriod:
            if self._data.rawOurActivePeriod:
                self._data.timeLeftInShift = 25 - self._shiftTimer.get()
            else:
                self._data.timeLeftInShift = (
                    25 + self._earlyTime - self._shiftTimer.get()
                )
        else:
            self._data.timeLeftInShift = 25 - self._shiftTimer.get() - self._earlyTime

        BraveLogger.pushSubsystemData(deepcopy(self._data))

    def startTeleop(self) -> None:
        self._teleopTimer.start()

    def overrideAutoWinner(self, newWinner: AutoWinner) -> None:
        self._winner = newWinner
        self._data.autoWinner = newWinner.value

    def flipAutoWinner(self) -> None:
        if self._winner == AutoWinner.OurAlliance:
            self.overrideAutoWinner(AutoWinner.TheirAlliance)
        elif self._winner == AutoWinner.TheirAlliance:
            self.overrideAutoWinner(AutoWinner.OurAlliance)

    def _isHubActive(self) -> bool:
        if self._winner == AutoWinner.Err:
            return True  # Err to shooting, not not shooting
        if not self._teleopTimer.hasElapsed(10):
            return True
        if self._winner == AutoWinner.OurAlliance and self._shiftCounter % 2 == 0:
            return True
        if self._winner == AutoWinner.TheirAlliance and self._shiftCounter % 2 == 1:
            return True
        if self._teleopTimer.get() > 110:  # endgame
            return True
        if self._shiftTimer.hasElapsed(
            25 - self._earlyTime
        ):  # within self._earlyTime seconds of shift change, the ball will make it in
            return True
        if not self._shiftTimer.hasElapsed(0.25):
            return True

        return False

    def isHubActive(self) -> bool:
        return self._data.ourActivePeriod
