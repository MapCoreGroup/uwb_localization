#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ground_truth_m', default_value='0.5'),
        DeclareLaunchArgument('condition', default_value='LOS'),
        DeclareLaunchArgument('num_samples', default_value='1000'),
        DeclareLaunchArgument('folder_name', default_value='0p5m'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    # Resolve launch arguments
    gt = LaunchConfiguration('ground_truth_m').perform(context)
    condition = LaunchConfiguration('condition').perform(context)
    n_samples = LaunchConfiguration('num_samples').perform(context)
    folder_name = LaunchConfiguration('folder_name').perform(context)

    # Base experiment directory under HOME
    record_base = os.path.join(
        os.path.expanduser('~'),
        'uwb_characterization',
        'record'
    )

    exp_dir = os.path.join(record_base, folder_name)
    bag_out = os.path.join(exp_dir, 'bag')      # rosbag will CREATE this
    res_dir = os.path.join(exp_dir, 'results')

    # -------- Commands --------

    # 1) Rosbag (START FIRST!)
    bag_cmd = (
        f'mkdir -p {exp_dir} && '
        'ros2 bag record '
        '/uwb/anchor1/distance_m '
        '/uwb/anchor1/raw '
        '/experiment/ground_truth_distance '
        '/experiment/condition '
        '/experiment/status '
        f'-o {bag_out}'
    )

    # 2) Anchor1 reader
    reader_cmd = [
        'ros2', 'run', 'uwb_serial_pub', 'anchor1'
    ]

    # 3) Characterization logger
    logger_cmd = [
        'ros2', 'run', 'uwb_serial_pub', 'uwb_characterization_logger',
        '--ros-args',
        '-p', f'ground_truth_m:={gt}',
        '-p', f'condition:={condition}',
        '-p', f'num_samples:={n_samples}',
        '-p', 'distance_topic:=/uwb/anchor1/distance_m',
        '-p', f'output_dir:={res_dir}',
        '-p', f'run_name:={folder_name}',
    ]

    return [
        ExecuteProcess(cmd=['bash', '-lc', bag_cmd], output='screen'),
        ExecuteProcess(cmd=reader_cmd, output='screen'),
        ExecuteProcess(cmd=logger_cmd, output='screen'),
    ]
