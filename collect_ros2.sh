#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export PATH="/usr/bin:$PATH"
exec /opt/ros/humble/bin/ros2 run piper_aio_ros2 collect "$@"
