#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 1, Chapter 2: Service Server
Description: Service to enable/disable robot motors with state tracking
Requirements: ROS 2 Jazzy, rclpy, std_srvs
Run: ros2 run humanoid_basics motor_control_service
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


class MotorControlService(Node):
    """
    Service server that controls motor enable state.

    This demonstrates:
    - Creating a service server with create_service()
    - Handling service requests in a callback
    - Maintaining state between service calls
    - Providing multiple services from one node
    """

    def __init__(self):
        super().__init__('motor_control_service')

        # Motor state
        self.motors_enabled = False
        self.enable_count = 0
        self.disable_count = 0

        # Create service to enable/disable motors
        self.enable_service = self.create_service(
            SetBool,
            '/motors/enable',
            self.enable_motors_callback
        )

        # Create service to get motor status
        self.status_service = self.create_service(
            Trigger,
            '/motors/status',
            self.status_callback
        )

        # Create service for emergency stop
        self.estop_service = self.create_service(
            Trigger,
            '/motors/emergency_stop',
            self.emergency_stop_callback
        )

        self.get_logger().info('Motor control services ready:')
        self.get_logger().info('  /motors/enable (SetBool)')
        self.get_logger().info('  /motors/status (Trigger)')
        self.get_logger().info('  /motors/emergency_stop (Trigger)')

    def enable_motors_callback(self, request, response):
        """
        Handle motor enable/disable requests.

        Args:
            request: SetBool.Request with data field (True=enable, False=disable)
            response: SetBool.Response to populate

        Returns:
            Populated response with success status and message
        """
        if request.data:
            # Enable motors
            self.motors_enabled = True
            self.enable_count += 1
            response.success = True
            response.message = f'Motors enabled (enable count: {self.enable_count})'
            self.get_logger().info('Motors ENABLED')
        else:
            # Disable motors
            self.motors_enabled = False
            self.disable_count += 1
            response.success = True
            response.message = f'Motors disabled (disable count: {self.disable_count})'
            self.get_logger().info('Motors DISABLED')

        return response

    def status_callback(self, request, response):
        """
        Return current motor status.

        Args:
            request: Trigger.Request (empty)
            response: Trigger.Response to populate

        Returns:
            Response with current motor state
        """
        status = 'ENABLED' if self.motors_enabled else 'DISABLED'
        response.success = True
        response.message = (
            f'Motors: {status} | '
            f'Enables: {self.enable_count} | '
            f'Disables: {self.disable_count}'
        )

        return response

    def emergency_stop_callback(self, request, response):
        """
        Handle emergency stop - immediately disable all motors.

        Args:
            request: Trigger.Request (empty)
            response: Trigger.Response to populate

        Returns:
            Response confirming emergency stop
        """
        was_enabled = self.motors_enabled
        self.motors_enabled = False

        if was_enabled:
            self.get_logger().error('EMERGENCY STOP - Motors disabled!')
            response.success = True
            response.message = 'Emergency stop executed - motors disabled'
        else:
            self.get_logger().warn('Emergency stop called but motors already disabled')
            response.success = True
            response.message = 'Motors were already disabled'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down motor control service')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
