#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 1, Chapter 2: Action Client
Description: Client that sends navigation goals and handles feedback
Requirements: ROS 2 Jazzy, rclpy, example_interfaces
Run: ros2 run humanoid_basics navigate_action_client
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from example_interfaces.action import Fibonacci


class NavigateActionClient(Node):
    """
    Action client for humanoid navigation.

    This demonstrates:
    - Creating an action client with ActionClient()
    - Sending goals asynchronously
    - Handling feedback callbacks
    - Processing final results
    - Canceling active goals
    """

    def __init__(self):
        super().__init__('navigate_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'navigate'
        )

        # Track goal state
        self._goal_handle = None
        self._goal_done = False

        self.get_logger().info('Navigate action client ready')

    def send_goal(self, num_steps: int, cancel_after: int = None):
        """
        Send a navigation goal to the server.

        Args:
            num_steps: Number of steps to navigate
            cancel_after: If set, cancel the goal after this many feedback messages
        """
        self._goal_done = False
        self._feedback_count = 0
        self._cancel_after = cancel_after

        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = num_steps

        self.get_logger().info(f'Sending goal: navigate {num_steps} steps')

        # Send goal with callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add callback for goal response
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the server's response to our goal request.

        Args:
            future: Future containing the goal response
        """
        self._goal_handle: ClientGoalHandle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            self._goal_done = True
            return

        self.get_logger().info('Goal accepted by server')

        # Get result asynchronously
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback from the action server.

        Args:
            feedback_msg: Feedback message from server
        """
        self._feedback_count += 1
        feedback = feedback_msg.feedback
        sequence_len = len(feedback.partial_sequence)

        self.get_logger().info(
            f'Feedback #{self._feedback_count}: '
            f'{sequence_len} steps completed'
        )

        # Optionally cancel after N feedback messages
        if self._cancel_after and self._feedback_count >= self._cancel_after:
            self.get_logger().info('Requesting goal cancellation...')
            self._goal_handle.cancel_goal_async()

    def get_result_callback(self, future):
        """
        Handle the final result from the action server.

        Args:
            future: Future containing the result
        """
        result = future.result()
        status = result.status

        # Interpret result status
        status_names = {
            1: 'ACCEPTED',
            2: 'EXECUTING',
            3: 'CANCELING',
            4: 'SUCCEEDED',
            5: 'CANCELED',
            6: 'ABORTED'
        }

        status_name = status_names.get(status, f'UNKNOWN({status})')

        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f'Navigation SUCCEEDED! '
                f'Final sequence: {result.result.sequence}'
            )
        elif status == 5:  # CANCELED
            self.get_logger().info(
                f'Navigation CANCELED. '
                f'Partial sequence: {result.result.sequence}'
            )
        else:
            self.get_logger().warn(f'Navigation ended with status: {status_name}')

        self._goal_done = True

    def is_done(self) -> bool:
        """Check if the current goal is complete."""
        return self._goal_done


def main(args=None):
    rclpy.init(args=args)
    client = NavigateActionClient()

    try:
        print('\n=== Navigation Action Demo ===\n')

        # Demo 1: Complete navigation
        print('--- Demo 1: Complete Navigation (10 steps) ---')
        client.send_goal(10)

        # Spin until goal is done
        while not client.is_done():
            rclpy.spin_once(client, timeout_sec=0.1)

        print()

        # Demo 2: Canceled navigation
        print('--- Demo 2: Canceled Navigation (cancel after 3 steps) ---')
        client.send_goal(10, cancel_after=3)

        while not client.is_done():
            rclpy.spin_once(client, timeout_sec=0.1)

        print('\n=== Demo Complete ===\n')

    except KeyboardInterrupt:
        client.get_logger().info('Demo interrupted')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
