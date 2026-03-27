#!/usr/bin/env bash
set -eo pipefail

source_safe() {
  # ROS setup scripts can reference unset optional vars.
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

# Build the current ROS 2 workspace (symlink install for fast iteration).
source_safe /opt/ros/humble/setup.bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

colcon build --symlink-install
source_safe "$SCRIPT_DIR/install/setup.bash"

echo "✓ Workspace built and sourced in: $SCRIPT_DIR"

