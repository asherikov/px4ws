"""Handles all communication with the PX4 autopilot using px4_msgs."""
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class PX4Communication:
    """PX4Communication."""

    def __init__(self, node):
        """
        Initialize the PX4 communication handler.

        Args:
            node: The ROS2 node that will handle the publishers and subscribers
        """
        self.node = node

        # Initialize variables
        self.vehicle_status = None
        self._last_nav_state = None

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_command_publisher = self.node.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', px4_qos)
        self.offboard_control_mode_publisher = self.node.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', px4_qos)
        self.trajectory_setpoint_publisher = self.node.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', px4_qos)

        self.vehicle_status_subscriber = self.node.create_subscription(
            VehicleStatus, 'fmu/out/vehicle_status_v1', self.vehicle_status_callback,
            px4_qos)

    def vehicle_status_callback(self, msg):
        """Update vehicle status."""
        self.vehicle_status = msg
        # Log navigation state changes
        if self._last_nav_state is None or self._last_nav_state != msg.nav_state:
            nav_states = {
                0: 'MANUAL',
                1: 'ALTCTL',
                2: 'POSCTL',
                3: 'AUTO_MISSION',
                4: 'AUTO_LOITER',
                5: 'AUTO_RTL',
                14: 'OFFBOARD',
                17: 'AUTO_TAKEOFF',
                18: 'AUTO_LAND'
            }
            current_state = nav_states.get(msg.nav_state, f'UNKNOWN({msg.nav_state})')
            self.node.get_logger().info(f'Navigation state changed to: {current_state}')
        self._last_nav_state = msg.nav_state

    # pylint: disable=unknown-option-value too-many-positional-arguments too-many-arguments
    def send_command(self, command,
                     param1=float('nan'), param2=float('nan'), param3=float('nan'), param4=float('nan'),
                     param5=float('nan'), param6=float('nan'), param7=float('nan')):
        """Send a vehicle command."""
        vehicle_command = VehicleCommand()
        vehicle_command.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        vehicle_command.command = command
        vehicle_command.param1 = param1
        vehicle_command.param2 = param2
        vehicle_command.param3 = param3
        vehicle_command.param4 = param4
        vehicle_command.param5 = param5
        vehicle_command.param6 = param6
        vehicle_command.param7 = param7
        vehicle_command.target_system = 1
        vehicle_command.target_component = 1
        vehicle_command.source_system = 1
        vehicle_command.source_component = 1
        vehicle_command.from_external = True

        self.vehicle_command_publisher.publish(vehicle_command)

    def arm_vehicle(self):
        """Send arm command to vehicle."""
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def set_offboard_mode(self):
        """Enable offboard control mode by publishing OffboardControlMode message."""
        # Send offboard control mode with velocity control enabled
        offboard_control_mode = OffboardControlMode()
        offboard_control_mode.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        offboard_control_mode.position = False
        offboard_control_mode.velocity = True  # We want to control velocity
        offboard_control_mode.acceleration = False
        offboard_control_mode.attitude = False
        offboard_control_mode.body_rate = False
        offboard_control_mode.thrust_and_torque = False
        offboard_control_mode.direct_actuator = False

        self.offboard_control_mode_publisher.publish(offboard_control_mode)

    def send_offboard_mode_command(self):
        """Send VEHICLE_CMD_DO_SET_MODE command to switch to offboard mode."""
        # Send command to set offboard mode (Base mode: 4, Custom main mode: 6)
        # Base mode: 4 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        # Custom main mode: 6 = PX4_CUSTOM_MAIN_MODE_OFFBOARD
        self.send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # Base mode (1=enable custom, 0=use system defaults)
            param2=6.0,  # Custom main mode (6 = OFFBOARD)
            param3=0.0   # Custom sub mode (not used for offboard)
        )

    def send_takeoff_command(self, altitude):
        """Send takeoff command to vehicle."""
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=altitude)

    def send_zero_velocity_setpoint(self):
        """Send zero velocity setpoint."""
        trajectory_setpoint = TrajectorySetpoint()
        trajectory_setpoint.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)

        # Set zero velocity
        trajectory_setpoint.velocity[0] = 0.0
        trajectory_setpoint.velocity[1] = 0.0
        trajectory_setpoint.velocity[2] = 0.0

        # Keep position and acceleration as NaN to not control them
        trajectory_setpoint.position[0] = float('nan')
        trajectory_setpoint.position[1] = float('nan')
        trajectory_setpoint.position[2] = float('nan')

        trajectory_setpoint.acceleration[0] = float('nan')
        trajectory_setpoint.acceleration[1] = float('nan')
        trajectory_setpoint.acceleration[2] = float('nan')

        trajectory_setpoint.yaw = float('nan')  # Don't control yaw
        trajectory_setpoint.yawspeed = float('nan')

        self.trajectory_setpoint_publisher.publish(trajectory_setpoint)

    def send_velocity_setpoint(self, velocity):
        """Send constant velocity setpoint."""
        trajectory_setpoint = TrajectorySetpoint()
        trajectory_setpoint.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)

        # Set constant velocity
        trajectory_setpoint.velocity[0] = float(velocity[0])
        trajectory_setpoint.velocity[1] = float(velocity[1])
        trajectory_setpoint.velocity[2] = float(velocity[2])

        # Keep position and acceleration as NaN to not control them
        trajectory_setpoint.position[0] = float('nan')
        trajectory_setpoint.position[1] = float('nan')
        trajectory_setpoint.position[2] = float('nan')

        trajectory_setpoint.acceleration[0] = float('nan')
        trajectory_setpoint.acceleration[1] = float('nan')
        trajectory_setpoint.acceleration[2] = float('nan')

        trajectory_setpoint.yaw = float('nan')  # Don't control yaw
        trajectory_setpoint.yawspeed = float('nan')

        self.trajectory_setpoint_publisher.publish(trajectory_setpoint)
