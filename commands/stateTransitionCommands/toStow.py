from typing import Callable

from commands2 import ParallelCommandGroup, SequentialCommandGroup, WaitUntilCommand
from commands2.command import InterruptionBehavior

from commands.baseCommands.intakeRetract import IntakeRetract
from commands.baseCommands.shooterStowHood import ShooterStowHood

from subsystems import Climber, Shooter, Turret


class ToStow(ParallelCommandGroup):
    def __init__(
        self,
        climber: Climber,
        shooter: Shooter,
        turret: Turret,
        inStowZone: Callable[[], bool],
    ):
        super().__init__()
        self.climber = climber
        self.shooter = shooter
        self.inZone = inStowZone
        self.addCommands(
            # SequentialCommandGroup(
            # ClimberStow(climber),
            # WaitUntilCommand(
            #     lambda: climber.getPositionInches() < 5
            #     and not climber.getHookDeployed()
            # ),
            # IntakeRetract(intake),
            # ),
            ShooterStowHood(shooter),
        )
        # self.addRequirements(turret)

    def isFinished(self) -> bool:
        return super().isFinished() and not self.inZone()

    # return (
    #     # (not self.intake.getDeployed())
    #     # and
    #     self.climber.getPositionInches() < 2
    #     and (not self.climber.getHookDeployed())
    #     and self.shooter.getHoodAngle().degrees() < 15
    # )

    def getInterruptionBehavior(self) -> InterruptionBehavior:
        return self.InterruptionBehavior.kCancelIncoming
