from commands2 import ParallelCommandGroup, SequentialCommandGroup, WaitUntilCommand

from commands.baseCommands.intakeRetract import IntakeRetract
from commands.baseCommands.shooterStowHood import ShooterStowHood
from commands.baseCommands.climberStow import ClimberStow

from subsystems import Intake, Climber, Shooter


class ToStow(ParallelCommandGroup):
    def __init__(self, intake: Intake, climber: Climber, shooter: Shooter):
        super().__init__()
        self.intake = intake
        self.climber = climber
        self.shooter = shooter
        self.addCommands(
            SequentialCommandGroup(
                ClimberStow(climber),
                WaitUntilCommand(
                    lambda: climber.getPositionInches() < 5
                    and not climber.getHookDeployed()
                ),
                IntakeRetract(intake),
            ),
            ShooterStowHood(shooter),
        )

    def isFinished(self) -> bool:
        return (
            (not self.intake.getDeployed())
            and self.climber.getPositionInches() < 5
            and (not self.climber.getHookDeployed())
            and self.shooter.getHoodAngle().degrees() < 15
        )
