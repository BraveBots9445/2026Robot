from dataclasses import dataclass

from time import sleep

from ntcore import NetworkTableInstance

from wpiutil.wpistruct import make_wpistruct


@make_wpistruct
@dataclass
class ShooterSetpointsStruct:
    distanceM: float
    flywheelVelocityRPM: float
    hoodAngledeg: float


kSERVER_ADDRESS_OPTIONS = ["10.94.45.2", "127.0.0.1"]

kSERVER_ADDRESS_IDX = 0

ntinst = NetworkTableInstance.getDefault()

ntinst.startClient4("TuningListener")
counter = 0
while not ntinst.isConnected():
    if counter == 0:
        print("Waiting to connect...")
    ntinst.setServer(kSERVER_ADDRESS_OPTIONS[kSERVER_ADDRESS_IDX])

    sleep(0.25)

    if not ntinst.isConnected():
        kSERVER_ADDRESS_IDX = (kSERVER_ADDRESS_IDX + 1) % len(kSERVER_ADDRESS_OPTIONS)
    counter = (counter + 1) % 20

nettable = ntinst.getTable("00ShooterTuneDistance")

listener = nettable.getStructArrayTopic("Setpoints", ShooterSetpointsStruct).subscribe(
    []
)

setpoints: list[ShooterSetpointsStruct] = []

print("Listening...")
print(f"Connected: {ntinst.isConnected()}")
prevData = listener.get()
while True:
    data = listener.get()
    if data != prevData:
        print(f"Received new setpoints w/ {len(data)} elements:")
        for setpoint in data:
            # print(
            #     f"Distance: {setpoint.distanceM} m, Flywheel Velocity: {setpoint.flywheelVelocityRPM} RPM, Hood Angle: {setpoint.hoodAngledeg} degrees"
            # )
            if not setpoint in setpoints:
                setpoints.append(setpoint)
        prevData = data
        print("-" * 50)
        orderedSetpointsList = sorted(
            list(setpoints), key=lambda setpoint: setpoint.distanceM
        )
        print(
            f"_distanceInterpArray = array({", ".join(str(setpoint.distanceM) for setpoint in orderedSetpointsList)})"
        )
        print(
            f"_hoodAngleInterpArray = array({", ".join(str(setpoint.hoodAngledeg) for setpoint in orderedSetpointsList)})"
        )
        print(
            f"_flywheelVelInterpArray = array({", ".join(str(setpoint.hoodAngledeg) for setpoint in orderedSetpointsList)})"
        )

        print("-" * 50)
