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
            'enter_offboard_mode',
            'arming_vehicle',
            'sending_takeoff',
            'enter_offboard_mode',  # Use the same state as the first one
            'performing_velocity_forward',
            'performing_velocity_zero',
            'performing_velocity_backward',
            'performing_landing',
            'finished'
        ]

        # Map state names to their corresponding handler functions
        self.state_handlers = {
            'waiting_for_status': self._handle_waiting_for_status,
            'enter_offboard_mode': self._handle_enter_offboard_mode,
            'arming_vehicle': self._handle_arming_vehicle,
            'sending_takeoff': self._handle_sending_takeoff,
            'performing_velocity_forward': self._handle_performing_velocity_forward,
            'performing_velocity_zero': self._handle_performing_velocity_zero,
            'performing_velocity_backward': self._handle_performing_velocity_backward,
            'performing_landing': self._handle_performing_landing,
            'finished': self._handle_finished,
        }


        # Index to track current position in the state sequence
        self.current_state_index = 0

        # State-specific variable storage (dynamically created and removed as needed)
        self.state_variables = {}

        # Store the previous state name for exit handling
        self.previous_state_name = None


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
            # Remove state variables from the map if they exist
            if current_state_name in self.state_variables:
                del self.state_variables[current_state_name]

            # Update the previous state name for reference
            self.previous_state_name = current_state_name

            # Advance to the next state in the sequence.
            self.node.get_logger().info(f'FSM: Switching from "{self.state_sequence[self.current_state_index]}" to "{self.state_sequence[self.current_state_index + 1]}"')
            self.current_state_index += 1

        # Return True if we've reached the final state and completed the sequence
        return self.current_state_index >= len(self.state_sequence) - 1 and current_state_name == 'finished'


    def _handle_waiting_for_status(self, current_time_sec):
        """Handle the waiting_for_status state, return whether a transition should occur."""
        if self.px4_comm.vehicle_status:
            # Initialize enter_offboard_mode variables inline
            if 'enter_offboard_mode' not in self.state_variables:
                self.state_variables['enter_offboard_mode'] = {
                    'start_time': 0,
                    'last_offboard_set_time': 0,
                    'offboard_set_interval': 0.2
                }
            # Set the start time for the offboard transition in the next state
            offboard_vars = self.state_variables['enter_offboard_mode']
            offboard_vars['start_time'] = current_time_sec
            offboard_vars['last_offboard_set_time'] = current_time_sec
            return True
        return False


    def _handle_enter_offboard_mode(self, current_time_sec):
        """Handle the enter_offboard_mode state, return whether a transition should occur."""
        # Get current state name
        current_state = self.state_sequence[self.current_state_index]

        # Initialize enter_offboard_mode variables inline for the current state
        if current_state not in self.state_variables:
            self.state_variables[current_state] = {
                'start_time': 0,
                'last_offboard_set_time': 0,
                'offboard_set_interval': 0.2  # seconds (200ms) to periodically try switching to offboard mode
            }

        offboard_vars = self.state_variables[current_state]

        # Initialize on first entry
        if offboard_vars['start_time'] == 0:
            offboard_vars['start_time'] = current_time_sec
            offboard_vars['last_offboard_set_time'] = current_time_sec

        last_offboard_set_time = offboard_vars['last_offboard_set_time']

        # Continue sending zero velocity commands while waiting for mode switch
        self.px4_comm.send_zero_velocity_setpoint()
        self.px4_comm.set_offboard_mode()  # Continue publishing offboard mode

        # Re-send VEHICLE_CMD_DO_SET_MODE command periodically
        offboard_set_interval = offboard_vars['offboard_set_interval']  # Get interval from state variables
        if current_time_sec - last_offboard_set_time > offboard_set_interval:
            self.px4_comm.send_offboard_mode_command()
            last_offboard_set_time = current_time_sec  # Update the time when command is sent

        # Update the stored time
        offboard_vars['last_offboard_set_time'] = last_offboard_set_time

        # Check if the vehicle has switched to offboard mode
        success = (self.px4_comm.vehicle_status
                   and self.px4_comm.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        if success:
            return True

        return False


    def _handle_arming_vehicle(self, current_time_sec):
        """Handle the arming_vehicle state, return whether a transition should occur."""
        # Initialize arming_vehicle variables inline
        if 'arming_vehicle' not in self.state_variables:
            self.state_variables['arming_vehicle'] = {'last_arm_command_time': None}

        # Continue sending zero velocity while arming
        self.px4_comm.send_zero_velocity_setpoint()
        self.px4_comm.set_offboard_mode()  # Continue publishing offboard mode

        # Check if vehicle is now armed
        if self.px4_comm.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            return True

        # Re-send arm command periodically with proper throttling
        last_arm_time = self.state_variables['arming_vehicle']['last_arm_command_time']
        if last_arm_time is None or current_time_sec - last_arm_time >= 1.0:  # Every second
            self.px4_comm.arm_vehicle()
            self.state_variables['arming_vehicle']['last_arm_command_time'] = current_time_sec  # Update the time when command is sent

        return False


    def _handle_sending_takeoff(self, current_time_sec):  # pylint: disable=unused-argument
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


    def _handle_performing_velocity_forward(self, current_time_sec):
        """Handle the forward velocity command state, return whether a transition should occur."""
        # Initialize velocity_forward variables inline
        if 'performing_velocity_forward' not in self.state_variables:
            self.state_variables['performing_velocity_forward'] = {
                'start_time': 0,
                'phase_start_time': 0
            }

        # Continue to send offboard control mode
        self.px4_comm.set_offboard_mode()

        # Initialize velocity command state variables on first entry
        vel_vars = self.state_variables['performing_velocity_forward']
        if vel_vars['phase_start_time'] == 0:
            vel_vars['start_time'] = current_time_sec
            vel_vars['phase_start_time'] = current_time_sec

        # Define parameters for the velocity pattern
        forward_duration = 4.0  # seconds for forward velocity
        forward_velocity = [2.0, 0.0, 0.0]  # m/s in x, y, z directions

        # Send forward velocity command
        self.px4_comm.send_velocity_setpoint(forward_velocity)

        # Print position every second while executing velocity commands
        if self.px4_comm.should_print_position(current_time_sec):
            position = self.px4_comm.get_current_position()
            if position:
                x, y, z = position
                self.node.get_logger().info(f'Vehicle position: x={x:.2f}, y={y:.2f}, z={z:.2f}')
            else:
                self.node.get_logger().info('Position unavailable')

        elapsed_time = current_time_sec - vel_vars['phase_start_time']
        if elapsed_time >= forward_duration:
            return True

        return False


    def _handle_performing_velocity_zero(self, current_time_sec):
        """Handle the zero velocity command state, return whether a transition should occur."""
        # Initialize velocity_zero variables inline
        if 'performing_velocity_zero' not in self.state_variables:
            self.state_variables['performing_velocity_zero'] = {
                'start_time': 0,
                'phase_start_time': 0
            }

        # Continue to send offboard control mode
        self.px4_comm.set_offboard_mode()

        # Initialize velocity command state variables on first entry
        vel_vars = self.state_variables['performing_velocity_zero']
        if vel_vars['phase_start_time'] == 0:
            vel_vars['start_time'] = current_time_sec
            vel_vars['phase_start_time'] = current_time_sec

        # Define parameters for the velocity pattern
        zero_duration = 2.0     # seconds for zero velocity
        zero_velocity = [0.0, 0.0, 0.0]     # m/s

        # Send zero velocity command
        self.px4_comm.send_velocity_setpoint(zero_velocity)

        # Print position every second while executing velocity commands
        if self.px4_comm.should_print_position(current_time_sec):
            position = self.px4_comm.get_current_position()
            if position:
                x, y, z = position
                self.node.get_logger().info(f'Vehicle position: x={x:.2f}, y={y:.2f}, z={z:.2f}')
            else:
                self.node.get_logger().info('Position unavailable')

        elapsed_time = current_time_sec - vel_vars['phase_start_time']
        if elapsed_time >= zero_duration:
            return True

        return False


    def _handle_performing_velocity_backward(self, current_time_sec):
        """Handle the backward velocity command state, return whether a transition should occur."""
        # Initialize velocity_backward variables inline
        if 'performing_velocity_backward' not in self.state_variables:
            self.state_variables['performing_velocity_backward'] = {
                'start_time': 0,
                'phase_start_time': 0
            }

        # Continue to send offboard control mode
        self.px4_comm.set_offboard_mode()

        # Initialize velocity command state variables on first entry
        vel_vars = self.state_variables['performing_velocity_backward']
        if vel_vars['phase_start_time'] == 0:
            vel_vars['start_time'] = current_time_sec
            vel_vars['phase_start_time'] = current_time_sec

        # Define parameters for the velocity pattern
        backward_duration = 4.0  # seconds for backward velocity
        backward_velocity = [-2.0, 0.0, 0.0]  # m/s in x, y, z directions

        # Send backward velocity command
        self.px4_comm.send_velocity_setpoint(backward_velocity)

        # Print position every second while executing velocity commands
        if self.px4_comm.should_print_position(current_time_sec):
            position = self.px4_comm.get_current_position()
            if position:
                x, y, z = position
                self.node.get_logger().info(f'Vehicle position: x={x:.2f}, y={y:.2f}, z={z:.2f}')
            else:
                self.node.get_logger().info('Position unavailable')

        elapsed_time = current_time_sec - vel_vars['phase_start_time']
        if elapsed_time >= backward_duration:
            return True

        return False


    def _handle_performing_landing(self, current_time_sec):  # pylint: disable=unused-argument
        """Handle the performing_landing state, return whether a transition should occur."""
        # Initialize performing_landing variables inline
        if 'performing_landing' not in self.state_variables:
            self.state_variables['performing_landing'] = {'landing_started': False}

        # Send landing command and monitor landing progress
        self.px4_comm.set_offboard_mode()

        # Check if landing is complete
        landing_started = self.state_variables['performing_landing']['landing_started']
        if self.px4_comm.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            if landing_started is True:
                return True
            self.px4_comm.send_landing_command()

        if landing_started is False:
            self.state_variables['performing_landing']['landing_started'] = True

        return False


    def _handle_finished(self, current_time_sec):  # pylint: disable=unused-argument
        """Handle the finished state."""
        # Final state after landing is complete
        # self.px4_comm.send_zero_velocity_setpoint()
        self.px4_comm.set_offboard_mode()

