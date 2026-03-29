from commands2 import RepeatCommand, SequentialCommandGroup

from subsystems.intake import Intake

from .IntakeSetPosition import IntakeSetPosition


class IntakeAgitate(RepeatCommand):
    def __init__(self, intake: Intake):
        """
        Repeatedly agitate the intake pivot through a fixed position sequence.

        Sequence: 60 -> 30 -> 45 -> 30 (degrees), repeated until interrupted.

        :param intake: Intake subsystem.
        :type intake: Intake
        """
        self.intake = intake
        self._startingPivotDegrees = 0.0

        sequence = SequentialCommandGroup(
            IntakeSetPosition(intake, 60.0),
            IntakeSetPosition(intake, 30.0),
            IntakeSetPosition(intake, 45.0),
            IntakeSetPosition(intake, 30.0),
        )

        super().__init__(sequence)
        self.setName("IntakeAgitate")

    def initialize(self):
        self._startingPivotDegrees = self.intake.getAngle().degrees()
        super().initialize()

    def end(self, interrupted: bool):
        super().end(interrupted)
        self.intake.setPivotSetpointDegrees(self._startingPivotDegrees)
