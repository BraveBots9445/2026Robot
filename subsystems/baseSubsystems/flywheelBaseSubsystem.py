from phoenix6.controls import VelocityDutyCycle, Follower
from phoenix6.signals import MotorAlignmentValue

from subsystems.baseSubsystems.rollerBaseSubsystem import *
from subsystems.baseSubsystems.rollerBaseSubsystem import RollerBaseSubsystem


@dataclass()
class FlywheelBaseSubsystemData(RollerBaseSubsystemData):
    motorRef: float

    @staticmethod
    def withRollerBaseSubsystemData(motorRef: float, parent: RollerBaseSubsystemData):
        return FlywheelBaseSubsystemData(
            parent.name,
            parent.velocity,
            parent.rotorVelocity,
            parent.statorCurrent,
            parent.motorDutyCycle,
            parent.motorTemp,
            parent.setpoint,
            parent.enabled,
            motorRef,
        )


class FlywheelBaseSubsystem(RollerBaseSubsystem):
    masterInfo: None | tuple[int, bool] = None

    def __init__(
        self,
        name: str,
        motorID: int,
        motorConfig: TalonFXConfiguration,
        simObject: FlywheelSim,
        canbus: str,
        *,
        motorToMechanismRatio: float = 1 / 1,
        dt: float = 0.02,
        enabled: bool = True
    ):
        super().__init__(
            name,
            motorID,
            motorConfig,
            simObject,
            canbus,
            motorToMechanismRatio=motorToMechanismRatio,
            dt=dt,
            enabled=enabled,
        )

        self._motorRefSignal = self._motor.get_closed_loop_reference(False)

    def periodic(self) -> FlywheelBaseSubsystemData:
        self._motorRefSignal.refresh()

        if self._enabled:
            if not self.masterInfo:
                self._motor.set_control(VelocityDutyCycle(self._setpoint))
            else:
                if self.masterInfo[1]:
                    alignment = MotorAlignmentValue.OPPOSED
                else:
                    alignment = MotorAlignmentValue.ALIGNED
                self._motor.set_control(Follower(self.masterInfo[0], alignment))
        else:
            self._motor.stopMotor()

        return FlywheelBaseSubsystemData.withRollerBaseSubsystemData(
            self._motorRefSignal.value, super().periodic()
        )

    def atSetpoint(self, tolerance: MechanismVelocity | float = 1) -> bool:
        return abs(self._setpoint - self._velocitySignal.value) < tolerance

    def addSlave(self, slave: FlywheelBaseSubsystem, invertedFromMaster: bool) -> None:  # type: ignore # we want special setup for flywheels that does not match the parent.
        # it is not expected to slave a roller to a flywheel or the opposite
        slave.masterInfo = (self._motor.device_id, invertedFromMaster)
        return super().addSlave(slave)
