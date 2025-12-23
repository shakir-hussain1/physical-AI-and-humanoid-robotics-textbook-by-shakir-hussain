#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 1, Chapter 1: Introduction to ROS 2
Description: A simple "Hello World" ROS 2 node that demonstrates basic node creation
Requirements: ROS 2 Jazzy, rclpy
Run: ros2 run my_first_package hello_ros2
"""

import rclpy
from rclpy.node import Node


class HelloROS2Node(Node):
    """
    A simple ROS 2 node that prints a greeting message.

    This demonstrates:
    - Creating a node class that inherits from rclpy.node.Node
    - Using timers to execute callbacks periodically
    - Basic logging with the ROS 2 logger
    """

    def __init__(self):
        # Initialize the node with the name 'hello_ros2'
        # This name will appear in ros2 node list
        super().__init__('hello_ros2')

        # Create a timer that calls timer_callback every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Initialize a counter to track how many times we've said hello
        self.counter = 0

        # Log that the node has started
        self.get_logger().info('Hello ROS 2 node has started!')

    def timer_callback(self):
        """
        Called every 1.0 seconds by the timer.
        Logs a greeting message with an incrementing counter.
        """
        self.counter += 1
        self.get_logger().info(f'Hello, Humanoid World! Message #{self.counter}')


def main(args=None):
    """
    Main entry point for the node.

    Steps:
    1. Initialize the rclpy library
    2. Create an instance of our node
    3. Spin (keep the node running and processing callbacks)
    4. Clean up when done
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of our node
    node = HelloROS2Node()

    try:
        # Keep the node running until Ctrl+C is pressed
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle clean shutdown on Ctrl+C
        node.get_logger().info('Shutting down Hello ROS 2 node...')
    finally:
        # Clean up: destroy the node and shutdown rclpy
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
