from commands2 import RepeatCommand, SequentialCommandGroup, WaitCommand

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
            WaitCommand(0.1),
            IntakeSetPosition(intake, 30.0),
            WaitCommand(0.05),
            IntakeSetPosition(intake, 45.0),
            WaitCommand(0.1),
            IntakeSetPosition(intake, 30.0),
            WaitCommand(0.05),
        )

        super().__init__(sequence)
        self.setName("IntakeAgitate")

    def initialize(self):
        self._startingPivotDegrees = self.intake.getAngle().degrees()
        self.intake.setRollerSetpoint(0.7)
        super().initialize()

    def end(self, interrupted: bool):
        super().end(interrupted)
        self.intake.setPivotSetpointDegrees(self._startingPivotDegrees)
        self.intake.setRollerSetpoint(0)
