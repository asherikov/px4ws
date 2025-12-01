"""Dedicated class for state machine logic."""
from px4_msgs.msg import VehicleStatus


class PX4TakeoffStateMachine:
    """PX4TakeoffStateMachine."""

    # pylint: disable=too-many-instance-attributes

    def __init__(self, node, px4_comm, config=None):
        """
        Initialize the state machine.

        Args:
            node: The ROS2 node
            px4_comm: The PX4 communication handler
            config: Configuration dictionary loaded from YAML file
        """
        self.node = node
        self.px4_comm = px4_comm
        self.config = config or {}


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
                    'offboard_set_interval': self.config['offboard_set_interval']
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
                'offboard_set_interval': self.config['offboard_set_interval']  # seconds to periodically try switching to offboard mode
            }

        offboard_vars = self.state_variables[current_state]

        # Initialize on first entry
        if offboard_vars['start_time'] == 0:
            offboard_vars['start_time'] = current_time_sec
            offboard_vars['last_offboard_set_time'] = current_time_sec

        # Continue sending zero velocity commands while waiting for mode switch
        self.px4_comm.send_zero_velocity_setpoint()
        self.px4_comm.set_offboard_mode()  # Continue publishing offboard mode

        # Re-send VEHICLE_CMD_DO_SET_MODE command periodically
        if current_time_sec - offboard_vars['last_offboard_set_time'] > offboard_vars['offboard_set_interval']:
            self.px4_comm.send_offboard_mode_command()
            offboard_vars['last_offboard_set_time'] = current_time_sec  # Update the time when command is sent

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

        # Re-send arm command periodically with proper throttling using config
        last_arm_time = self.state_variables['arming_vehicle']['last_arm_command_time']
        if last_arm_time is None or current_time_sec - last_arm_time >= self.config['arm_command_interval']:
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
            # Send takeoff command with altitude from config
            self.px4_comm.send_takeoff_command(self.config['takeoff_altitude'])

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

        # Send forward velocity command
        self.px4_comm.send_velocity_setpoint(self.config['velocity_commands']['forward']['velocity'])

        # Print position based on interval from config while executing velocity commands
        if self.px4_comm.should_print_position(current_time_sec, self.config['position_print_interval']):
            position = self.px4_comm.get_current_position()
            if position:
                x, y, z = position
                self.node.get_logger().info(f'Vehicle position: x={x:.2f}, y={y:.2f}, z={z:.2f}')
            else:
                self.node.get_logger().info('Position unavailable')

        elapsed_time = current_time_sec - vel_vars['phase_start_time']
        if elapsed_time >= self.config['velocity_commands']['forward']['duration']:
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

        # Send zero velocity command
        self.px4_comm.send_velocity_setpoint(self.config['velocity_commands']['zero']['velocity'])

        # Print position based on interval from config while executing velocity commands
        if self.px4_comm.should_print_position(current_time_sec, self.config['position_print_interval']):
            position = self.px4_comm.get_current_position()
            if position:
                x, y, z = position
                self.node.get_logger().info(f'Vehicle position: x={x:.2f}, y={y:.2f}, z={z:.2f}')
            else:
                self.node.get_logger().info('Position unavailable')

        elapsed_time = current_time_sec - vel_vars['phase_start_time']
        if elapsed_time >= self.config['velocity_commands']['zero']['duration']:
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

        # Send backward velocity command
        self.px4_comm.send_velocity_setpoint(self.config['velocity_commands']['backward']['velocity'])

        # Print position based on interval from config while executing velocity commands
        if self.px4_comm.should_print_position(current_time_sec, self.config['position_print_interval']):
            position = self.px4_comm.get_current_position()
            if position:
                x, y, z = position
                self.node.get_logger().info(f'Vehicle position: x={x:.2f}, y={y:.2f}, z={z:.2f}')
            else:
                self.node.get_logger().info('Position unavailable')

        elapsed_time = current_time_sec - vel_vars['phase_start_time']
        if elapsed_time >= self.config['velocity_commands']['backward']['duration']:
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

