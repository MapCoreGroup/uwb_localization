# UWB Localization ROS2 Workspace

ROS2 workspace for Ultra-Wideband (UWB) localization system.

## Packages

### uwb_serial_pub
UWB anchor serial reader nodes for reading range measurements from UWB anchors via serial communication.

**Features:**
- Serial communication with multiple UWB anchors (anchor1-4)
- Range measurement publishing
- Particle filter for localization
- Bayesian direction estimation
- Multi-anchor circle visualization
- Range statistics and characterization

**Nodes:**
- `anchor1`, `anchor2`, `anchor3`, `anchor4` - Serial readers for each anchor
- `multi_anchor_circles` - Visualization of anchor ranges as circles
- `uwb_pf_node` - Particle filter node for position estimation
- `uwb_bayes_direction` - Bayesian direction estimation
- `range_stats_node` - Range statistics calculation
- `uwb_characterization_logger` - Data logging for system characterization

### uwb_viz
UWB radius visualization package for RViz.

**Features:**
- Real-time visualization of UWB anchor ranges in RViz
- Static transform handling for anchor positions

## Requirements

- ROS2 (tested with Humble/Humble)
- Python 3
- Required ROS2 packages:
  - `rclpy`
  - `std_msgs`
  - `geometry_msgs`
  - `visualization_msgs`
  - `tf2_ros`
  - `rviz2`

## Building

```bash
cd /path/to/uwb_localization
colcon build
source install/setup.bash
```

## Usage

### Launch multi-anchor visualization:
```bash
ros2 launch uwb_serial_pub multi_anchor_circles.launch.py
```

### Launch UWB characterization:
```bash
ros2 launch uwb_serial_pub uwb_characterization.launch.py
```

### Launch UWB visualization:
```bash
ros2 launch uwb_viz static_tf.launch.py
```

## License

MIT License

## Maintainer

ilan
