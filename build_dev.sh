#!/usr/bin/env bash
set -euo pipefail

# Build the current ROS 2 workspace (symlink install for fast iteration).
source /opt/ros/humble/setup.bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

colcon build --symlink-install
source "$SCRIPT_DIR/install/setup.bash"

echo "✓ Workspace built and sourced in: $SCRIPT_DIR"

