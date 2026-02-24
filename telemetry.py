from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger, swerve, units
from wpilib import Color, Color8Bit, Mechanism2d, MechanismLigament2d, SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState


class Telemetry:
    def __init__(self, max_speed: units.meters_per_second):
        """
        Construct a telemetry object with the specified max speed of the robot.

        :param max_speed: Maximum speed
        :type max_speed: units.meters_per_second
        """
        self._max_speed = max_speed
        SignalLogger.start()

        # What to publish over networktables for telemetry
        self._inst = NetworkTableInstance.getDefault()

        # Robot swerve drive state
        self._drive_state_table = self._inst.getTable("000Drivetrain")
        self._drive_pose = self._drive_state_table.getStructTopic(
            "Pose", Pose2d
        ).publish()
        self._drive_speeds = self._drive_state_table.getStructTopic(
            "Speeds", ChassisSpeeds
        ).publish()
        self._drive_module_states = self._drive_state_table.getStructArrayTopic(
            "ModuleStates", SwerveModuleState
        ).publish()
        self._drive_module_targets = self._drive_state_table.getStructArrayTopic(
            "ModuleTargets", SwerveModuleState
        ).publish()
        self._drive_module_positions = self._drive_state_table.getStructArrayTopic(
            "ModulePositions", SwerveModulePosition
        ).publish()
        self._drive_timestamp = self._drive_state_table.getDoubleTopic(
            "Timestamp"
        ).publish()
        self._drive_odometry_frequency = self._drive_state_table.getDoubleTopic(
            "OdometryFrequency"
        ).publish()

        # Robot pose for field positioning
        self._table = self._inst.getTable("Pose")
        self._field_pub = self._table.getDoubleArrayTopic("robotPose").publish()
        self._field_type_pub = self._table.getStringTopic(".type").publish()

        # Mechanisms to represent the swerve module states
        self._module_mechanisms: list[Mechanism2d] = [
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
        ]
        # A direction and length changing ligament for speed representation
        self._module_speeds: list[MechanismLigament2d] = [
            self._module_mechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
        ]
        # A direction changing and length constant ligament for module direction
        self._module_directions: list[MechanismLigament2d] = [
            self._module_mechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
        ]

        # Set up the module state Mechanism2d telemetry
        for i, module_mechanism in enumerate(self._module_mechanisms):
            SmartDashboard.putData(f"Module {i}", module_mechanism)

    def __init_telemetry_buffers(self):
        """Pre-allocate reusable arrays for telemetry to avoid per-cycle allocation."""
        self._pose_array = [0.0, 0.0, 0.0]
        self._module_states_array = [0.0] * 8
        self._module_targets_array = [0.0] * 8
        self._telem_cycle = 0

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger.
        """
        # Lazy-init buffers on first call
        if not hasattr(self, '_pose_array'):
            self.__init_telemetry_buffers()

        # Telemeterize the swerve drive state
        self._drive_pose.set(state.pose)
        self._drive_speeds.set(state.speeds)
        self._drive_module_states.set(state.module_states)
        self._drive_module_targets.set(state.module_targets)
        self._drive_module_positions.set(state.module_positions)
        self._drive_timestamp.set(state.timestamp)
        self._drive_odometry_frequency.set(1.0 / state.odometry_period)

        # Reuse pre-allocated arrays instead of creating new ones every cycle
        self._pose_array[0] = state.pose.x
        self._pose_array[1] = state.pose.y
        self._pose_array[2] = state.pose.rotation().degrees()

        for i in range(4):
            idx = i * 2
            self._module_states_array[idx] = state.module_states[i].angle.radians()
            self._module_states_array[idx + 1] = state.module_states[i].speed
            self._module_targets_array[idx] = state.module_targets[i].angle.radians()
            self._module_targets_array[idx + 1] = state.module_targets[i].speed

        SignalLogger.write_double_array("DriveState/Pose", self._pose_array)
        SignalLogger.write_double_array("DriveState/ModuleStates", self._module_states_array)
        SignalLogger.write_double_array(
            "DriveState/ModuleTargets", self._module_targets_array
        )
        SignalLogger.write_double(
            "DriveState/OdometryPeriod", state.odometry_period, "seconds"
        )

        # Telemeterize the pose to a Field2d
        self._field_type_pub.set("Field2d")
        self._field_pub.set(self._pose_array)

        # Telemeterize module states to Mechanism2d — skip every other cycle to save time
        self._telem_cycle += 1
        if self._telem_cycle % 2 == 0:
            for i, module_state in enumerate(state.module_states):
                angle_deg = module_state.angle.degrees()
                self._module_speeds[i].setAngle(angle_deg)
                self._module_directions[i].setAngle(angle_deg)
                self._module_speeds[i].setLength(module_state.speed / (2 * self._max_speed))
