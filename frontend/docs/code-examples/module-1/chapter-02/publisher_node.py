#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 1, Chapter 2: Publisher Node
Description: Publishes simulated humanoid joint states at 50 Hz
Requirements: ROS 2 Jazzy, rclpy, sensor_msgs
Run: ros2 run humanoid_basics joint_state_publisher
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePublisher(Node):
    """
    Publishes simulated joint states for a humanoid robot arm.

    This demonstrates:
    - Creating a publisher with create_publisher()
    - Using timers for periodic publishing
    - Populating sensor_msgs/JointState messages
    - Simulating joint motion with sinusoidal functions
    """

    def __init__(self):
        super().__init__('joint_state_publisher')

        # Declare parameters
        self.declare_parameter('publish_rate', 50.0)
        publish_rate = self.get_parameter('publish_rate').value

        # Create publisher for joint states
        # Queue size of 10 allows some buffering if subscriber is slow
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Create timer for periodic publishing
        timer_period = 1.0 / publish_rate  # Convert Hz to seconds
        self.timer = self.create_timer(timer_period, self.publish_joint_states)

        # Time counter for simulation
        self.time = 0.0
        self.dt = timer_period

        # Define joint names for humanoid arm
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_roll_joint',
            'wrist_pitch_joint',
            'gripper_joint'
        ]

        self.get_logger().info(
            f'Joint state publisher started at {publish_rate} Hz'
        )

    def publish_joint_states(self):
        """
        Publish simulated joint positions.

        Creates sinusoidal motion for each joint with different
        frequencies and amplitudes to simulate realistic arm movement.
        """
        # Create message
        msg = JointState()

        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Set joint names
        msg.name = self.joint_names

        # Calculate simulated positions using sinusoidal motion
        # Each joint moves at a different frequency and amplitude
        msg.position = [
            math.sin(self.time * 0.5) * 1.0,      # shoulder_pan: ±1.0 rad
            math.sin(self.time * 0.3) * 0.5,      # shoulder_lift: ±0.5 rad
            math.sin(self.time * 0.7) * 1.2,      # elbow: ±1.2 rad
            math.sin(self.time * 1.0) * 0.8,      # wrist_roll: ±0.8 rad
            math.sin(self.time * 0.8) * 0.6,      # wrist_pitch: ±0.6 rad
            abs(math.sin(self.time * 0.4)) * 0.04 # gripper: 0-0.04 m
        ]

        # Calculate velocities (derivative of position)
        msg.velocity = [
            math.cos(self.time * 0.5) * 0.5 * 1.0,
            math.cos(self.time * 0.3) * 0.3 * 0.5,
            math.cos(self.time * 0.7) * 0.7 * 1.2,
            math.cos(self.time * 1.0) * 1.0 * 0.8,
            math.cos(self.time * 0.8) * 0.8 * 0.6,
            0.0  # Gripper velocity not simulated
        ]

        # Effort can be left empty or simulated
        msg.effort = []

        # Publish the message
        self.publisher.publish(msg)

        # Increment time
        self.time += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down joint state publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
