#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 1, Chapter 2: Service Client
Description: Client that calls motor control services
Requirements: ROS 2 Jazzy, rclpy, std_srvs
Run: ros2 run humanoid_basics motor_control_client
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


class MotorControlClient(Node):
    """
    Service client for motor control.

    This demonstrates:
    - Creating service clients with create_client()
    - Waiting for service availability
    - Calling services synchronously and asynchronously
    - Handling service responses
    """

    def __init__(self):
        super().__init__('motor_control_client')

        # Create clients for each service
        self.enable_client = self.create_client(SetBool, '/motors/enable')
        self.status_client = self.create_client(Trigger, '/motors/status')
        self.estop_client = self.create_client(Trigger, '/motors/emergency_stop')

        # Wait for services to be available
        self.get_logger().info('Waiting for motor services...')

        services = [
            (self.enable_client, '/motors/enable'),
            (self.status_client, '/motors/status'),
            (self.estop_client, '/motors/emergency_stop')
        ]

        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'  Waiting for {name}...')

        self.get_logger().info('All motor services available!')

    def enable_motors(self, enable: bool) -> tuple:
        """
        Enable or disable motors.

        Args:
            enable: True to enable, False to disable

        Returns:
            Tuple of (success, message)
        """
        request = SetBool.Request()
        request.data = enable

        # Synchronous call using spin_until_future_complete
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        return (response.success, response.message)

    def get_status(self) -> tuple:
        """
        Get current motor status.

        Returns:
            Tuple of (success, message)
        """
        request = Trigger.Request()

        future = self.status_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        return (response.success, response.message)

    def emergency_stop(self) -> tuple:
        """
        Execute emergency stop.

        Returns:
            Tuple of (success, message)
        """
        request = Trigger.Request()

        future = self.estop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        return (response.success, response.message)


def main(args=None):
    rclpy.init(args=args)
    client = MotorControlClient()

    try:
        # Demonstrate service calls
        print('\n=== Motor Control Demo ===\n')

        # Get initial status
        success, message = client.get_status()
        print(f'Initial status: {message}')

        # Enable motors
        success, message = client.enable_motors(True)
        print(f'Enable motors: {message}')

        # Get status after enable
        success, message = client.get_status()
        print(f'Status: {message}')

        # Disable motors
        success, message = client.enable_motors(False)
        print(f'Disable motors: {message}')

        # Enable again
        success, message = client.enable_motors(True)
        print(f'Enable motors: {message}')

        # Emergency stop
        success, message = client.emergency_stop()
        print(f'Emergency stop: {message}')

        # Final status
        success, message = client.get_status()
        print(f'Final status: {message}')

        print('\n=== Demo Complete ===\n')

    except Exception as e:
        client.get_logger().error(f'Error: {e}')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
