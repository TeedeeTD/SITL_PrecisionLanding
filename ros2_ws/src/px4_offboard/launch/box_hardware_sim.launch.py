#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Paths ---
    box_manager_config_path = PathJoinSubstitution([
        FindPackageShare('box_manager'),
        'config',
        'box_state_manager.yaml'
    ])

    # --- Declare Arguments ---
    box_id_arg = DeclareLaunchArgument(
        'box_id',
        default_value='1',
        description='Box ID'
    )

    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1',
        description='Drone ID'
    )

    # --- Nodes ---
    # 1. Mock Box Hardware
    mock_box_hardware_node = Node(
        package='px4_offboard',
        executable='mock_box_hardware',
        name='mock_box_hardware',
        output='screen',
        parameters=[
            {'box_id': LaunchConfiguration('box_id')}
        ]
    )

    # 2. MAVROS to DIB Telemetry
    mavros_to_dib_telemetry_node = Node(
        package='px4_offboard',
        executable='mavros_to_dib_telemetry',
        name='mavros_to_dib_telemetry',
        output='screen',
        parameters=[
            {'drone_id': LaunchConfiguration('drone_id')}
        ]
    )

    # 3. Box State Manager (from box_manager package)
    box_state_manager_node = Node(
        package='box_manager',
        executable='box_state_manager_node',
        name='box_state_manager',
        output='screen',
        parameters=[
            box_manager_config_path,
            {'box_id': LaunchConfiguration('box_id')}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # --- Launch Description ---
    return LaunchDescription([
        box_id_arg,
        drone_id_arg,
        mock_box_hardware_node,
        mavros_to_dib_telemetry_node,
        box_state_manager_node
    ])
