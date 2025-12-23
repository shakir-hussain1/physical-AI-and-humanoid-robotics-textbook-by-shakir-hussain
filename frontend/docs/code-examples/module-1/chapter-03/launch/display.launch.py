#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 1, Chapter 3: URDF Visualization Launch
Description: Launch file to visualize humanoid URDF in RViz2 with joint controls
Requirements: ROS 2 Jazzy, robot_state_publisher, joint_state_publisher_gui, rviz2
Run: ros2 launch humanoid_description display.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for humanoid URDF visualization.

    This launch file:
    1. Loads the humanoid URDF
    2. Starts robot_state_publisher to publish TF transforms
    3. Starts joint_state_publisher_gui for interactive joint control
    4. Launches RViz2 with a pre-configured display
    """

    # ========== Launch Arguments ==========

    # Use GUI for joint state publisher
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Use joint_state_publisher_gui instead of joint_state_publisher'
    )

    # URDF file path (can be overridden)
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='',
        description='Path to URDF file (uses default if empty)'
    )

    # ========== Get Configurations ==========

    use_gui = LaunchConfiguration('gui')

    # Get URDF file path
    # In a real package, this would use get_package_share_directory
    # For this example, we construct a relative path
    default_urdf = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'humanoid_urdf',
        'humanoid.urdf'
    )

    # Read URDF content
    # Note: In production, use xacro.process_file for .xacro files
    with open(default_urdf, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    robot_description = {'robot_description': robot_description_content}

    # ========== Nodes ==========

    # Robot State Publisher
    # Publishes TF transforms based on joint states and URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Joint State Publisher GUI
    # Provides sliders to control each joint
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )

    # Joint State Publisher (non-GUI version)
    # Use this for automated testing or when GUI is not needed
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(['not ', use_gui])
    )

    # RViz2
    # Visualization tool for robot models, TF frames, sensor data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # You can specify a config file:
        # arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')]
    )

    # ========== Return Launch Description ==========

    return LaunchDescription([
        # Arguments
        gui_arg,
        urdf_file_arg,

        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node,
    ])


# Allow running this file directly for testing
if __name__ == '__main__':
    generate_launch_description()
