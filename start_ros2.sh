#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export PYTHONPATH="${PIPER_SDK_PATH:-$ROOT/../piper_sdk}:${PYTHONPATH:-}"
export PATH="/usr/bin:$PATH"
exec /opt/ros/humble/bin/ros2 launch piper_aio_ros2 four_can.launch.py "$@"
