#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('uwb_serial_pub')
    default_params = os.path.join(pkg_share, 'config', 'uwb_pf_params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to uwb_pf_node parameter YAML file.',
        ),
        Node(
            package='uwb_serial_pub',
            executable='uwb_pf_node',
            name='uwb_pf_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
