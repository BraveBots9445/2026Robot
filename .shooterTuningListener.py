from dataclasses import dataclass

from ntcore import NetworkTableInstance

from wpiutil.wpistruct import make_wpistruct


@make_wpistruct
@dataclass
class ShooterSetpointsStruct:
    distanceM: float
    flywheelVelocityRPM: float
    hoodAngledeg: float


# kSERVER_ADDRESS = "10.94.45.2"
kSERVER_ADDRESS = "10.94.45.2"

ntinst = NetworkTableInstance.getDefault()
ntinst.startClient4("TuningListener")
ntinst.setServer(kSERVER_ADDRESS)

print(ntinst.getConnections())

nettable = ntinst.getTable("00ShooterTuneDistance")

listener = nettable.getStructArrayTopic("Setpoints", ShooterSetpointsStruct).subscribe(
    []
)

distanceSet = {}
hoodAngleSet = {}
flywheelVelSet = {}

print("Listening...")
prevData = listener.get()
while True:
    data = listener.get()
    if data != prevData:
        print("Received new setpoints:")
        for setpoint in data:
            print(
                f"Distance: {setpoint.distanceM} m, Flywheel Velocity: {setpoint.flywheelVelocityRPM} RPM, Hood Angle: {setpoint.hoodAngledeg} degrees"
            )
        prevData = data
        print("-" * 50)
        print(
            f"_distanceInterpArray = array([{', '.join(str(setpoint.distanceM) for setpoint in data)}])\n"
        )
        print(
            f"_hoodAngleInterpArray = array([{', '.join(str(setpoint.hoodAngledeg) for setpoint in data)}])\n"
        )
        print(
            f"_flywheelVelInterpArray = array([{', '.join(str(setpoint.flywheelVelocityRPM) for setpoint in data)}])\n"
        )
        print("-" * 50)
