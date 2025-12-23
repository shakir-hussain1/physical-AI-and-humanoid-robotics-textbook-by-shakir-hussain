#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 1, Chapter 2: Launch File Example
Description: Launches a complete humanoid sensor pipeline with multiple nodes
Requirements: ROS 2 Jazzy, humanoid_basics package
Run: ros2 launch humanoid_basics sensor_pipeline.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Generate launch description for the humanoid sensor pipeline.

    This launch file demonstrates:
    - Declaring launch arguments with defaults
    - Creating multiple node instances
    - Using namespaces for node organization
    - Conditional node launching
    - Setting node parameters from launch
    - Remapping topics
    """

    # ========== Launch Arguments ==========

    # Logging level argument
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    # Publish rate for joint states
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Joint state publish rate in Hz'
    )

    # Enable motor control service
    enable_motor_service_arg = DeclareLaunchArgument(
        'enable_motor_service',
        default_value='true',
        description='Whether to launch motor control service'
    )

    # Robot namespace
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='humanoid',
        description='Namespace for all nodes'
    )

    # ========== Get Launch Configurations ==========

    log_level = LaunchConfiguration('log_level')
    publish_rate = LaunchConfiguration('publish_rate')
    enable_motor_service = LaunchConfiguration('enable_motor_service')
    namespace = LaunchConfiguration('namespace')

    # ========== Node Definitions ==========

    # Joint state publisher node
    joint_publisher_node = Node(
        package='humanoid_basics',
        executable='joint_state_publisher',
        name='joint_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate')
        }],
        arguments=['--ros-args', '--log-level', log_level],
        # Remap topics if needed
        remappings=[
            # ('original_topic', 'new_topic'),
        ]
    )

    # Joint state subscriber node
    joint_subscriber_node = Node(
        package='humanoid_basics',
        executable='joint_state_subscriber',
        name='joint_subscriber',
        namespace=namespace,
        output='screen',
        parameters=[{
            'velocity_threshold': 1.0
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Motor control service (conditional)
    motor_service_node = Node(
        package='humanoid_basics',
        executable='motor_control_service',
        name='motor_control',
        namespace=namespace,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(enable_motor_service)
    )

    # Navigate action server
    navigate_server_node = Node(
        package='humanoid_basics',
        executable='navigate_action_server',
        name='navigate_server',
        namespace=namespace,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )

    # ========== Grouped Actions ==========

    # Group all sensor nodes together
    sensor_nodes = GroupAction([
        LogInfo(msg=['Launching sensor pipeline in namespace: ', namespace]),
        joint_publisher_node,
        joint_subscriber_node,
    ])

    # Group control nodes
    control_nodes = GroupAction([
        LogInfo(msg='Launching control nodes...'),
        motor_service_node,
        navigate_server_node,
    ])

    # ========== Return Launch Description ==========

    return LaunchDescription([
        # Declare arguments first
        log_level_arg,
        publish_rate_arg,
        enable_motor_service_arg,
        namespace_arg,

        # Log startup
        LogInfo(msg='=== Humanoid Sensor Pipeline ==='),
        LogInfo(msg=['Log level: ', log_level]),
        LogInfo(msg=['Publish rate: ', publish_rate, ' Hz']),

        # Launch node groups
        sensor_nodes,
        control_nodes,

        # Final log
        LogInfo(msg='=== Pipeline Started ==='),
    ])
