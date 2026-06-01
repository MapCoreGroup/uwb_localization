from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import yaml


def _rviz_env():
    env = {"QT_QPA_PLATFORM": "xcb"}
    if "LD_LIBRARY_PATH" in os.environ:
        env["LD_LIBRARY_PATH"] = os.environ["LD_LIBRARY_PATH"]
    return env


def make_nodes(context, *args, **kwargs):
    cfg_path = LaunchConfiguration("config").perform(context)
    rviz_cfg = LaunchConfiguration("rviz_config").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)

    # Load YAML with anchor definitions (frame IDs + xyz).
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    fixed = cfg.get("fixed_frame", "map")
    actions = []

    anchors = cfg.get("anchors", []) or []
    if len(anchors) != 4:
        # Not fatal, but it typically indicates a configuration mismatch.
        print(f"[static_tf.launch] Expected 4 anchors, got {len(anchors)}")

    # 1) Static TFs: parent = fixed frame, child = each anchor frame_id.
    for a in anchors:
        name = a.get("name", "anchor")
        frame_id = a.get("frame_id", f"anchor_{name}_frame")
        x, y, z = a.get("xyz", [0.0, 0.0, 0.0])

        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"static_tf_{frame_id}",
                arguments=[
                    str(x),
                    str(y),
                    str(z),
                    "0",
                    "0",
                    "0",
                    fixed,
                    frame_id,
                ],
                output="screen",
            )
        )

    # 2) Radius visualization node.
    actions.append(
        Node(
            package="uwb_viz",
            executable="radius_viz",
            name="radius_viz",
            output="screen",
            parameters=[{"config_path": cfg_path}],
        )
    )

    # 3) RViz2 viewer (visual inspection during experiments).
    if use_rviz.lower() in ("true", "1", "yes", "on"):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_cfg],
                output="screen",
                env=_rviz_env(),
            )
        )

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory("uwb_viz")
    default_cfg = os.path.join(pkg_share, "config", "anchors.yaml")
    default_rviz_cfg = os.path.join(pkg_share, "rviz", "uwb_viz.rviz")
    try:
        pf_pkg_share = get_package_share_directory("uwb_serial_pub")
        default_rviz_cfg = os.path.join(
            pf_pkg_share, "rviz", "multi_anchor_circles.rviz"
        )
    except Exception:
        pass
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "RCUTILS_CONSOLE_OUTPUT_FORMAT",
                "[{severity}] {message}",
            ),
            DeclareLaunchArgument("config", default_value=default_cfg),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_cfg),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 (requires GUI display).",
            ),
            OpaqueFunction(function=make_nodes),
        ]
    )

