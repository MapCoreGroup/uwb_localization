#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('uwb_serial_pub')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'multi_anchor_circles.rviz')
    urdf_path = os.path.join(pkg_share, 'urdf', 'anchors.urdf')
    
    # Read URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Preserve LD_LIBRARY_PATH for rviz2 to find OGRE libraries
    rviz_env = {'QT_QPA_PLATFORM': 'xcb'}
    if 'LD_LIBRARY_PATH' in os.environ:
        rviz_env['LD_LIBRARY_PATH'] = os.environ['LD_LIBRARY_PATH']
    
    return LaunchDescription([
        # Robot State Publisher for URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # Anchor 1 serial reader
        Node(
            package='uwb_serial_pub',
            executable='uwb_serial_reader_node',
            name='anchor1_reader',
            output='screen',
            parameters=[{
                'anchor_id': 1,
                'baudrate': 115200,
            }]
        ),
        
        # Anchor 2 serial reader
        Node(
            package='uwb_serial_pub',
            executable='uwb_serial_reader_node',
            name='anchor2_reader',
            output='screen',
            parameters=[{
                'anchor_id': 2,
                'baudrate': 115200,
            }]
        ),
        
        # Anchor 3 serial reader
        Node(
            package='uwb_serial_pub',
            executable='uwb_serial_reader_node',
            name='anchor3_reader',
            output='screen',
            parameters=[{
                'anchor_id': 3,
                'baudrate': 115200,
            }]
        ),
        
        # Anchor 4 serial reader
        Node(
            package='uwb_serial_pub',
            executable='uwb_serial_reader_node',
            name='anchor4_reader',
            output='screen',
            parameters=[{
                'anchor_id': 4,
                'baudrate': 115200,
            }]
        ),
        
        # Multi anchor circles visualization
        Node(
            package='uwb_serial_pub',
            executable='multi_anchor_circles',
            name='multi_anchor_circles',
            output='screen'
        ),
        
        # RViz2 visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            env=rviz_env,  # Preserves LD_LIBRARY_PATH for OGRE libraries
            respawn=True  # Restart if crashes
        ),
    ])
