#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 1, Chapter 2: Subscriber Node
Description: Subscribes to joint states and logs joint positions
Requirements: ROS 2 Jazzy, rclpy, sensor_msgs
Run: ros2 run humanoid_basics joint_state_subscriber
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointStateSubscriber(Node):
    """
    Subscribes to joint states and processes them.

    This demonstrates:
    - Creating a subscription with create_subscription()
    - Processing incoming messages in a callback
    - Calculating derived values from sensor data
    - Using ROS 2 logging at different severity levels
    """

    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Declare parameters
        self.declare_parameter('velocity_threshold', 1.0)
        self.velocity_threshold = self.get_parameter('velocity_threshold').value

        # Create subscription
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Track statistics
        self.message_count = 0
        self.last_positions = {}

        self.get_logger().info('Joint state subscriber started')

    def joint_state_callback(self, msg: JointState):
        """
        Process received joint state messages.

        This callback is invoked every time a message is received
        on the /joint_states topic.
        """
        self.message_count += 1

        # Log joint positions periodically (every 50 messages)
        if self.message_count % 50 == 0:
            self.log_joint_summary(msg)

        # Check for high velocities
        self.check_velocity_limits(msg)

        # Store current positions for comparison
        for name, position in zip(msg.name, msg.position):
            self.last_positions[name] = position

    def log_joint_summary(self, msg: JointState):
        """Log a summary of all joint positions."""
        self.get_logger().info('--- Joint State Summary ---')

        for i, name in enumerate(msg.name):
            position_deg = math.degrees(msg.position[i])
            velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0

            self.get_logger().info(
                f'  {name}: {position_deg:+.1f}° ({msg.position[i]:+.3f} rad) '
                f'vel: {velocity:+.3f} rad/s'
            )

    def check_velocity_limits(self, msg: JointState):
        """Check if any joint velocities exceed threshold."""
        if not msg.velocity:
            return

        for name, velocity in zip(msg.name, msg.velocity):
            if abs(velocity) > self.velocity_threshold:
                self.get_logger().warn(
                    f'High velocity on {name}: {velocity:.3f} rad/s '
                    f'(threshold: {self.velocity_threshold})'
                )


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Shutting down. Received {node.message_count} messages.'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
