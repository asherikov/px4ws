"""ROS2 node that interacts with PX4-Autopilot using px4_msgs."""

import os

import ament_index_python

import rclpy
from rclpy.node import Node

import yaml

from .px4_communication import PX4Communication
from .px4_state_machine import PX4TakeoffStateMachine


class PX4OffboardDemoNode(Node):
    """PX4OffboardDemoNode."""

    def __init__(self):
        """PX4OffboardDemoNode."""
        super().__init__('demo_node')

        # Load configuration from YAML file
        # First try to find config file using ament resource system
        config_path = None
        try:
            package_share_path = ament_index_python.packages.get_package_share_directory('px4_offboard_demo_py')
            config_path = os.path.join(package_share_path, 'config', 'demo_config.yaml')
        except (ImportError, ament_index_python.packages.PackageNotFoundError):
            # Fallback to the old method if ament_index_python is not available
            config_path = os.path.join(os.path.dirname(__file__), 'config', 'demo_config.yaml')

        try:
            with open(config_path, 'r', encoding='utf-8') as file:
                self.config = yaml.safe_load(file)
            self.get_logger().info(f'Configuration loaded from: {config_path}')
        except FileNotFoundError as exception:
            self.get_logger().error(f'Configuration file not found: {config_path}')
            raise RuntimeError(f'Configuration file not found: {config_path}') from exception

        # Initialize PX4 communication handler (includes subscriptions)
        self.px4_comm = PX4Communication(self)

        # Initialize state machine with configuration
        self.state_machine = PX4TakeoffStateMachine(self, self.px4_comm, self.config)

        # Create timer for sending messages based on config
        timer_period = 1.0 / self.config['timer_frequency']  # Convert frequency to period
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('PX4 Offboard Demo Node initialized')
        # Initialize takeoff_started for backward compatibility
        self.takeoff_started = False
        # Flag to indicate if state machine has completed
        self.completed = False


    def timer_callback(self):
        """Timer callback that delegates to the state machine."""
        if self.state_machine.run_state_machine():
            self.get_logger().info('State machine sequence completed, shutting down node')
            # Cancel the timer to stop further callbacks
            self.timer.cancel()
            # Set completion flag to signal main function to shut down
            self.completed = True
            # Destroy the node to clean up resources
            self.destroy_node()


def main(args=None):
    """start."""
    rclpy.init(args=args)

    demo_node = PX4OffboardDemoNode()

    try:
        # Use spin_once in a loop to check for completion, using timeout from config
        while rclpy.ok() and not getattr(demo_node, 'completed', False):
            rclpy.spin_once(demo_node, timeout_sec=demo_node.config['timeout_sec'])
    except KeyboardInterrupt:
        demo_node.get_logger().info('Interrupted by user')
    finally:
        # Only try to destroy node if it's not already destroyed
        if demo_node.handle is not None:
            demo_node.destroy_node()
        # Only shutdown if rclpy is still ok
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
