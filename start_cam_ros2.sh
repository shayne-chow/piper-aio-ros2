#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
CAMERA_SETUP="${CAMERA_ROS2_SETUP:-$ROOT/camera_ws/install_ros2/setup.bash}"
if [[ ! -f "$CAMERA_SETUP" ]]; then
  echo "Camera ROS2 installation missing: $CAMERA_SETUP. See docs/ROS2_4CAN.md." >&2
  exit 1
fi
source "$CAMERA_SETUP"
source "$ROOT/ros2_ws/install/setup.bash"
export PATH="/usr/bin:$PATH"
exec /opt/ros/humble/bin/ros2 launch piper_aio_ros2 cameras.launch.py "$@"
