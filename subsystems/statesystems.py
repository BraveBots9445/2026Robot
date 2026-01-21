from enum import Enum, auto 

class IntakeState(Enum): 
    OFF = auto 
    Intaking = auto()
    Holding = auto() 
    Enjecting = auto() 

    import wpilib 
    from wpilib.interfaces import MotorController

    class IntakeSubsystem(wpilib.Subsystembase):

        def __init__(self, motor: MotorController, beam_break):
            super().__init__()
            self.motor = motor 
            self.beam_break = beam_break 
            self.state = IntakeState.OFF
            self.state_start_time = wpilib.Timer.getFPGATimestamp() 

        def set_state(self, new_state: IntakeState): 
            if new_state != self.state: 
                wpilib.smartdashboard.putstring( 
                    "Intake/State", new_state.name 
                )
                self.state = new_state
                self.state_start_time = wpilib.Timer.getFPGATimestamp() 
                self.on_enter_state(new_state)

            def on_enter_state(self, state: IntakeState): 
                if state == IntakeState.OFF: 
                    self.motor.stopMotor() 

                elif state == IntakeState.Intaking: 
                    self.motor.set(0.7)
                
                elif state == IntakeState.Holding: 
                    self.motor.set(0.1) 
                
                elif state == IntakeState.Enjecting:
                    self.motor.set(-0.8)

               def periodic(self): 
                elapsed = (
                    wpilib.Timer.getFPGATimestamp() - self.state_start_time
                )      

                if self.state == IntakeState.Intaking: 
                    if self.beam_break.get(): 
                        self.set_state(IntakeState.Holding)

                elif self.state == IntakeState.Enjecting: 
                    if elapsed > 0.5: 
                        self.set_state(IntakeState.OFF)

                import commands2 

                class StarIntake(commands2.InstantCommand): 
                    def __init__(self, intake: IntakeSubsystem):
                        super().__init__() 
                        self.intake = intake 
                        self.addRequirements(intake)


                    def initialize(self):
                        self.intake.set_state(IntakeState.Intaking)         


                        self.driverController.a().onTrue(
                            StartIntake(self.intake),
                        ) 


                        commands2.SequentialCommandGroup( 
                            StarIntake(intake), 
                           commands2.WaitCommand(1.0), 
                           ShootCommand(shooter)
                        ) 


                        def disabledInit(self):
                            self.set_state(IntakeState.OFF)


                            wpilib.SmartDashboard.putString(
                                "Intake/State", self.state.name
                            )