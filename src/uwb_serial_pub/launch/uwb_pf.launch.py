#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('uwb_serial_pub')
    default_params = os.path.join(pkg_share, 'config', 'uwb_pf_params.yaml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'anchors.urdf')

    with open(urdf_path, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()

    params_file = LaunchConfiguration('params_file')
    use_robot_state = LaunchConfiguration('use_robot_state_publisher')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to uwb_pf_node parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'use_robot_state_publisher',
            default_value='true',
            description='Publish anchors URDF (needed for RViz TF).',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            condition=IfCondition(use_robot_state),
        ),
        Node(
            package='uwb_serial_pub',
            executable='uwb_serial_reader_node',
            name='anchor1_reader',
            output='screen',
            parameters=[{'anchor_id': 1, 'baudrate': 115200}],
        ),
        Node(
            package='uwb_serial_pub',
            executable='uwb_serial_reader_node',
            name='anchor2_reader',
            output='screen',
            parameters=[{'anchor_id': 2, 'baudrate': 115200}],
        ),
        Node(
            package='uwb_serial_pub',
            executable='uwb_serial_reader_node',
            name='anchor3_reader',
            output='screen',
            parameters=[{'anchor_id': 3, 'baudrate': 115200}],
        ),
        Node(
            package='uwb_serial_pub',
            executable='uwb_serial_reader_node',
            name='anchor4_reader',
            output='screen',
            parameters=[{'anchor_id': 4, 'baudrate': 115200}],
        ),
        Node(
            package='uwb_serial_pub',
            executable='uwb_pf_node',
            name='uwb_pf_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
