"""ROS2 node that interacts with PX4-Autopilot using px4_msgs."""

import rclpy
from rclpy.node import Node

from .px4_communication import PX4Communication
from .px4_state_machine import PX4TakeoffStateMachine


class PX4OffboardDemoNode(Node):
    """PX4OffboardDemoNode."""

    def __init__(self):
        """PX4OffboardDemoNode."""
        super().__init__('demo_node')

        # Initialize PX4 communication handler (includes subscriptions)
        self.px4_comm = PX4Communication(self)

        # Initialize state machine
        self.state_machine = PX4TakeoffStateMachine(self, self.px4_comm)

        # Create timer for sending messages (50 Hz to send constant zero velocity)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz for faster updates

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
        # Use spin_once in a loop to check for completion
        while rclpy.ok() and not getattr(demo_node, 'completed', False):
            rclpy.spin_once(demo_node, timeout_sec=0.1)
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
