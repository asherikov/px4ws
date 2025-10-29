"""Dedicated class for state machine logic."""
from px4_msgs.msg import VehicleStatus


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


        # Define the state sequence for the complete flight operation
        self.state_sequence = [
            'waiting_for_status',
            'sending_zero_velocity',
            'setting_offboard_mode',
            'arming_vehicle',
            'sending_takeoff',
            'performing_landing',
            'finished'
        ]

        # Map state names to their corresponding handler functions
        self.state_handlers = {
            'waiting_for_status': self._handle_waiting_for_status,
            'sending_zero_velocity': self._handle_sending_zero_velocity,
            'setting_offboard_mode': self._handle_setting_offboard_mode,
            'arming_vehicle': self._handle_arming_vehicle,
            'sending_takeoff': self._handle_sending_takeoff,
            'performing_landing': self._handle_performing_landing,
            'finished': self._handle_finished,
        }

        # Index to track current position in the state sequence
        self.current_state_index = 0

        # State machine variables
        self.zero_velocity_start_time = 0
        self.zero_velocity_duration = 2.0  # seconds to send zero velocity before offboard mode
        self.last_offboard_set_time = 0
        self.last_arm_command_time = None  # Track time of last arm command to implement proper throttling
        self.landing_started = False


    def run_state_machine(self):
        """Run state machine execution function."""
        current_time_sec = self.node.get_clock().now().nanoseconds / 1e9

        # Get the current state name from the sequence
        if self.current_state_index >= len(self.state_sequence):
            raise IndexError('State index out of bounds')

        current_state_name = self.state_sequence[self.current_state_index]

        # Look up the handler for the current state
        if current_state_name not in self.state_handlers:
            raise ValueError(f'No handler found for state: {current_state_name}')

        handler = self.state_handlers[current_state_name]

        # Execute the handler and check if we should advance to the next state
        if handler(current_time_sec) and self.current_state_index < len(self.state_sequence) - 1:
            # Advance to the next state in the sequence.
            self.node.get_logger().info(f'FSM: Switching from "{self.state_sequence[self.current_state_index]}" to "{self.state_sequence[self.current_state_index+1]}"')
            self.current_state_index += 1

        # Return True if we've reached the final state and completed the sequence
        return self.current_state_index >= len(self.state_sequence) - 1 and current_state_name == 'finished'

    def _handle_waiting_for_status(self, current_time_sec):
        """Handle the waiting_for_status state, return whether a transition should occur."""
        if self.px4_comm.vehicle_status:
            # Set the start time for the zero velocity duration
            self.zero_velocity_start_time = current_time_sec
            return True
        return False

    def _handle_sending_zero_velocity(self, current_time_sec):
        """Handle the sending_zero_velocity state, return whether a transition should occur."""
        # Send zero velocity and offboard control mode for 2 seconds to switch to offboard control
        self.px4_comm.set_offboard_mode()
        self.px4_comm.send_zero_velocity_setpoint()

        elapsed_time = current_time_sec - self.zero_velocity_start_time
        if elapsed_time >= self.zero_velocity_duration:
            # Send the VEHICLE_CMD_DO_SET_MODE command to switch to offboard mode
            self.px4_comm.send_offboard_mode_command()
            return True

        return False

    def _handle_setting_offboard_mode(self, current_time_sec):
        """Handle the setting_offboard_mode state, return whether a transition should occur."""
        # Continue sending zero velocity commands while waiting for mode switch
        self.px4_comm.send_zero_velocity_setpoint()

        # Keep sending offboard mode commands regardless
        self.px4_comm.set_offboard_mode()
        # Re-send VEHICLE_CMD_DO_SET_MODE command periodically
        if current_time_sec - self.last_offboard_set_time > 1.0:
            self.px4_comm.send_offboard_mode_command()
            self.last_offboard_set_time = current_time_sec  # Update the time when command is sent

        # Check if the vehicle has switched to offboard mode
        if self.px4_comm.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return True

        return False

    def _handle_arming_vehicle(self, current_time_sec):
        """Handle the arming_vehicle state, return whether a transition should occur."""
        # Continue sending zero velocity while arming
        self.px4_comm.send_zero_velocity_setpoint()
        self.px4_comm.set_offboard_mode()  # Continue publishing offboard mode

        # Check if vehicle is now armed
        if (self.px4_comm.vehicle_status and
            self.px4_comm.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            return True

        # Re-send arm command periodically with proper throttling
        if self.last_arm_command_time is None or current_time_sec - self.last_arm_command_time >= 1.0:  # Every second
            self.px4_comm.arm_vehicle()
            self.last_arm_command_time = current_time_sec  # Update the time when command is sent

        return False

    def _handle_sending_takeoff(self, current_time_sec): # pylint: disable=unused-argument
        """Handle the sending_takeoff state, return whether a transition should occur."""
        # Continue to send offboard control mode
        self.px4_comm.set_offboard_mode()

        # Check if takeoff is complete
        if (self.px4_comm.vehicle_status.nav_state in [VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
                                                      VehicleStatus.NAVIGATION_STATE_POSCTL]):
            return True

        if self.px4_comm.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            # Send takeoff command
            self.px4_comm.send_takeoff_command(10.0)

        return False

    def _handle_performing_landing(self, current_time_sec): # pylint: disable=unused-argument
        """Handle the performing_landing state, return whether a transition should occur."""
        # Send landing command and monitor landing progress
        self.px4_comm.set_offboard_mode()

        # Check if landing is complete
        if self.px4_comm.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            if self.landing_started is True:
                return True
            self.px4_comm.send_landing_command()

        if self.landing_started is False:
            self.landing_started = True

        return False

    def _handle_finished(self, current_time_sec): # pylint: disable=unused-argument
        """Handle the finished state."""
        # Final state after landing is complete
        # self.px4_comm.send_zero_velocity_setpoint()
        self.px4_comm.set_offboard_mode()
