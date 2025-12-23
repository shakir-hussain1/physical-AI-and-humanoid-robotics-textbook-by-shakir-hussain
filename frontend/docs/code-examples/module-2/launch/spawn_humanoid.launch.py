#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 2, Chapter 6: Spawn Humanoid Launch File

Description: Launch file to spawn humanoid robot in Gazebo Harmonic
             with ROS 2 bridge for joint states and transforms.

Requirements:
    - ROS 2 Jazzy
    - Gazebo Harmonic (ros-jazzy-ros-gz)
    - humanoid_description package from Module 1

Run: ros2 launch humanoid_gazebo spawn_humanoid.launch.py
     ros2 launch humanoid_gazebo spawn_humanoid.launch.py world:=obstacle_world.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for spawning humanoid in Gazebo."""

    # ==================== Package Paths ====================
    pkg_humanoid_description = get_package_share_directory('humanoid_description')
    pkg_humanoid_gazebo = get_package_share_directory('humanoid_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # ==================== Launch Arguments ====================

    # World file to load
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.sdf',
        description='World file name (must be in humanoid_gazebo/worlds/)'
    )

    # Robot spawn position
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='X position to spawn robot'
    )

    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Y position to spawn robot'
    )

    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.85',
        description='Z position to spawn robot (above ground)'
    )

    # Use simulation time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # Start RViz
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 for visualization'
    )

    # ==================== Configurations ====================

    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    # Full path to world file
    world_path = PathJoinSubstitution([
        pkg_humanoid_gazebo, 'worlds', world
    ])

    # URDF file path
    urdf_file = os.path.join(
        pkg_humanoid_description, 'urdf', 'humanoid.urdf'
    )

    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # ==================== Environment Variables ====================

    # Set Gazebo resource path to find models
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_humanoid_description, '..')
    )

    # ==================== Nodes ====================

    # Start Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot state publisher - publishes TF and robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    # Spawn robot entity in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_humanoid',
        output='screen',
        arguments=[
            '-name', 'humanoid',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
    )

    # ROS-Gazebo bridge for common topics
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            # Clock synchronization
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Joint states from Gazebo to ROS
            '/world/empty_world/model/humanoid/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',

            # Pose ground truth
            '/model/humanoid/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
        ],
    )

    # Optional RViz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    # ==================== Return Launch Description ====================

    return LaunchDescription([
        # Environment
        gz_resource_path,

        # Arguments
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        use_sim_time_arg,
        rviz_arg,

        # Nodes
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        rviz_node,
    ])


# Allow running this file directly for testing
if __name__ == '__main__':
    generate_launch_description()
