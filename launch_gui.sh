#!/bin/bash
export QT_QPA_PLATFORM=xcb
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${REMOTE_PI_WS:-$SCRIPT_DIR}"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
ros2 run remote_pi_pkg auv_control
