from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml, os

def make_nodes(context, *args, **kwargs):
    cfg_path = LaunchConfiguration('config').perform(context)

    # טען YAML בבטחה
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f) or {}

    fixed = cfg.get('fixed_frame', 'map')
    actions = []

    # 1) Static TFs
    for a in cfg.get('anchors', []):
        name = a.get('name', 'anchor')
        frame_id = a.get('frame_id', f'anchor_{name}')
        x, y, z = a.get('xyz', [0.0, 0.0, 0.0])
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{name}',
            arguments=[str(x), str(y), str(z), '0', '0', '0', fixed, frame_id],
            output='screen'
        ))

    # 2) Radius viz node
    actions.append(Node(
        package='uwb_viz',
        executable='radius_viz',
        name='radius_viz',
        output='screen',
        parameters=[{'config_path': cfg_path}]
    ))

    # 3) RViz (עם סביבה נקייה)
    pkg_share = get_package_share_directory('uwb_viz')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'uwb_viz.rviz')
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        env={'QT_QPA_PLATFORM': 'xcb', 'LD_LIBRARY_PATH': ''}  # מנקה זיהום מסנאפ/Wayland
    ))

    return actions

def generate_launch_description():
    default_cfg = os.path.join(os.path.dirname(__file__), '..', 'config', 'anchors.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}'),
        DeclareLaunchArgument('config', default_value=default_cfg),
        OpaqueFunction(function=make_nodes),
    ])
