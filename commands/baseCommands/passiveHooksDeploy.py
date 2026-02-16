from commands2 import Command

from subsystems import PassiveHooks


class PassiveHooksDeploy(Command):
    def __init__(self, passiveHooks: PassiveHooks) -> None:
        self.passiveHooks = passiveHooks
        self.addRequirements(passiveHooks)

    def initialize(self):
        self.passiveHooks.deploy()
