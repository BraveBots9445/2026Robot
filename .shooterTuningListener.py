from dataclasses import dataclass

from ntcore import NetworkTableInstance

from wpiutil.wpistruct import make_wpistruct


@make_wpistruct
@dataclass
class ShooterSetpointsStruct:
    distanceM: float
    flywheelVelocityRPM: float
    hoodAngledeg: float


kSERVER_ADDRESS = "10.94.45.2"

ntinst = NetworkTableInstance.getDefault()
ntinst.startClient4("TuningListener")
ntinst.setServer(kSERVER_ADDRESS)

nettable = ntinst.getTable("00ShooterTuneDistance")

listener = nettable.getStructArrayTopic("Setpoints", ShooterSetpointsStruct).subscribe(
    []
)

prevData = listener.get()
while True:
    data = listener.get()
    if data != prevData:
        print("Received new setpoints:")
        for setpoint in data:
            print(
                f"Distance: {setpoint.distance} m, Flywheel Velocity: {setpoint.flywheelVelocity} m/s, Hood Angle: {setpoint.hoodAngle} degrees"
            )
        prevData = data
