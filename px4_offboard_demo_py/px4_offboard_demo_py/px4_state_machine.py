"""Dedicated class for state machine logic."""
from px4_msgs.msg import VehicleStatus

import rclpy


class PX4TakeoffStateMachine:
    """PX4TakeoffStateMachine."""

    def __init__(self, node, px4_comm):
        """
        Initialize the state machine.

        Args:
            node: The ROS2 node
            px4_comm: The PX4 communication handler
        """
        self.node = node
        self.px4_comm = px4_comm


        # State machine variables
        # waiting_for_status, sending_zero_velocity, setting_offboard_mode,
        # waiting_for_offboard, arming_vehicle, waiting_for_offboard,
        # sending_takeoff, waiting_takeoff_completion
        self.state = 'waiting_for_status'
        self.zero_velocity_start_time = 0
        self.zero_velocity_duration = 2.0  # seconds to send zero velocity before offboard mode
        self.last_offboard_set_time = 0
        self.last_arm_command_time = None  # Track time of last arm command to implement proper throttling


    def run_state_machine(self):
        """Run state machine execution function."""
        current_time_sec = self.node.get_clock().now().nanoseconds / 1e9

        if self.state == 'waiting_for_status':
            # Wait to receive the first vehicle status before proceeding
            if self.px4_comm.vehicle_status:
                self.node.get_logger().info('Received vehicle status, proceeding with initialization')
                self.state = 'sending_zero_velocity'
                # Set the start time for the zero velocity duration
                self.zero_velocity_start_time = current_time_sec
                self.node.get_logger().info('Starting zero velocity and offboard control commands')
            else:
                self.node.get_logger().log('Waiting for initial vehicle status...', rclpy.logging.LoggingSeverity.INFO, throttle_duration_sec=2.0)

        elif self.state == 'sending_zero_velocity':
            # Send zero velocity and offboard control mode for 2 seconds to switch to offboard control
            self.px4_comm.set_offboard_mode()
            self.px4_comm.send_zero_velocity_setpoint()

            elapsed_time = current_time_sec - self.zero_velocity_start_time
            if elapsed_time >= self.zero_velocity_duration:
                self.node.get_logger().info('Zero velocity and offboard control sent for required duration, switching to offboard mode using VEHICLE_CMD_DO_SET_MODE')
                self.state = 'setting_offboard_mode'
                # Send the VEHICLE_CMD_DO_SET_MODE command to switch to offboard mode
                self.px4_comm.send_offboard_mode_command()
            else:
                self.node.get_logger().log('Sending zero velocity and offboard control', rclpy.logging.LoggingSeverity.INFO, throttle_duration_sec=2.0)

        elif self.state == 'setting_offboard_mode':
            # Continue sending zero velocity commands while waiting for mode switch
            self.px4_comm.send_zero_velocity_setpoint()

            # Wait until the vehicle switches to offboard mode
            if self.px4_comm.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.node.get_logger().info('Successfully switched to offboard mode, proceeding to arming')
                self.state = 'arming_vehicle'
            else:
                # Keep sending zero velocity and ensure offboard mode is set
                self.px4_comm.set_offboard_mode()
                # Re-send VEHICLE_CMD_DO_SET_MODE command periodically
                if current_time_sec - self.last_offboard_set_time > 1.0:
                    self.px4_comm.send_offboard_mode_command()
                    self.last_offboard_set_time = current_time_sec  # Update the time when command is sent

                nav_state_str = self.px4_comm.vehicle_status.nav_state if self.px4_comm.vehicle_status else 'Unknown'
                self.node.get_logger().log(f'Waiting for offboard mode, current state: {nav_state_str}', rclpy.logging.LoggingSeverity.INFO, throttle_duration_sec=2.0)

        elif self.state == 'arming_vehicle':
            # Continue sending zero velocity while arming
            self.px4_comm.send_zero_velocity_setpoint()
            self.px4_comm.set_offboard_mode()  # Continue publishing offboard mode

            # Check if vehicle is now armed
            if self.px4_comm.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.node.get_logger().info('Vehicle successfully armed, proceeding to takeoff')
                self.state = 'sending_takeoff'
            else:
                # Re-send arm command periodically with proper throttling
                if self.last_arm_command_time is None or current_time_sec - self.last_arm_command_time >= 1.0:  # Every second
                    self.px4_comm.arm_vehicle()
                    self.last_arm_command_time = current_time_sec  # Update the time when command is sent

                arming_state_str = 'Unknown' if not self.px4_comm.vehicle_status else (
                    'ARMED' if self.px4_comm.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED else 'DISARMED'
                )
                self.node.get_logger().log(f'Waiting for arming, current arming state: {arming_state_str}', rclpy.logging.LoggingSeverity.INFO, throttle_duration_sec=2.0)

        elif self.state == 'sending_takeoff':
            # Continue to send offboard control mode
            self.px4_comm.set_offboard_mode()

            # Wait for takeoff to complete
            if self.px4_comm.vehicle_status.nav_state in [VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
                                                          VehicleStatus.NAVIGATION_STATE_POSCTL]:
                self.node.get_logger().info('Takeoff completed')
                self.state = 'completed'
                self.node.get_logger().info('Takeoff sequence completed successfully')
            elif self.px4_comm.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                # Send takeoff command
                self.px4_comm.send_takeoff_command(10.0)

        elif self.state == 'completed':
            # After takeoff is completed, continue sending zero velocity`
            # self.px4_comm.send_zero_velocity_setpoint()
            # self.px4_comm.set_offboard_mode()
            self.node.get_logger().log('Takeoff completed, continuing to send zero velocity commands', rclpy.logging.LoggingSeverity.INFO, throttle_duration_sec=2.0)
