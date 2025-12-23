#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 1, Chapter 2: Action Server
Description: Action server for humanoid navigation with feedback
Requirements: ROS 2 Jazzy, rclpy, example_interfaces
Run: ros2 run humanoid_basics navigate_action_server
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.action import Fibonacci


class NavigateActionServer(Node):
    """
    Action server for humanoid navigation.

    This demonstrates:
    - Creating an action server with ActionServer()
    - Handling goal acceptance/rejection
    - Publishing feedback during execution
    - Supporting goal cancellation
    - Returning results on completion

    Note: Using Fibonacci action as a stand-in for a custom navigation action.
    In a real system, you would define a custom action type.
    """

    def __init__(self):
        super().__init__('navigate_action_server')

        # Use reentrant callback group for concurrent goal handling
        self._callback_group = ReentrantCallbackGroup()

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'navigate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group
        )

        # Track active goals
        self._current_goal = None

        self.get_logger().info('Navigate action server ready')
        self.get_logger().info('  Action: /navigate')
        self.get_logger().info('  Type: example_interfaces/action/Fibonacci')

    def goal_callback(self, goal_request):
        """
        Decide whether to accept or reject a goal.

        Args:
            goal_request: The incoming goal request

        Returns:
            GoalResponse.ACCEPT or GoalResponse.REJECT
        """
        self.get_logger().info(f'Received goal request: {goal_request.order} steps')

        # Validate goal
        if goal_request.order < 1:
            self.get_logger().warn('Rejecting goal: steps must be >= 1')
            return GoalResponse.REJECT

        if goal_request.order > 100:
            self.get_logger().warn('Rejecting goal: steps must be <= 100')
            return GoalResponse.REJECT

        # Accept valid goals
        self.get_logger().info('Goal accepted')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Decide whether to accept a cancellation request.

        Args:
            goal_handle: Handle to the goal being canceled

        Returns:
            CancelResponse.ACCEPT or CancelResponse.REJECT
        """
        self.get_logger().info('Received cancel request')

        # Always accept cancellation in this example
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execute the navigation action.

        This is the main execution loop that:
        1. Simulates navigation progress
        2. Publishes feedback at regular intervals
        3. Checks for cancellation requests
        4. Returns the final result

        Args:
            goal_handle: Handle to the active goal

        Returns:
            The action result
        """
        self.get_logger().info('Executing navigation...')

        # Create feedback message
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        total_steps = goal_handle.request.order

        # Execute navigation step by step
        for step in range(1, total_steps):
            # Check if cancellation was requested
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Navigation canceled')

                result = Fibonacci.Result()
                result.sequence = feedback_msg.partial_sequence
                return result

            # Simulate navigation step (500ms per step)
            time.sleep(0.5)

            # Update progress (Fibonacci sequence as demo)
            next_val = feedback_msg.partial_sequence[-1] + feedback_msg.partial_sequence[-2]
            feedback_msg.partial_sequence.append(next_val)

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Log progress
            progress = (step / total_steps) * 100
            self.get_logger().info(
                f'Step {step}/{total_steps} complete ({progress:.0f}%)'
            )

        # Navigation complete - mark goal as succeeded
        goal_handle.succeed()
        self.get_logger().info('Navigation completed successfully!')

        # Return final result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigateActionServer()

    # Use multi-threaded executor for concurrent goal handling
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down navigate action server')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
