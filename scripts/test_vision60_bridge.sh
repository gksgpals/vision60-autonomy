#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMP_WS="$(mktemp -d /private/tmp/vision60_bridge_test.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

mkdir -p "${TEMP_WS}/src"
cp -R \
  "${WORKSPACE_ROOT}/src/vision60_msgs" \
  "${WORKSPACE_ROOT}/src/vision60_bridge" \
  "${WORKSPACE_ROOT}/src/vision60_bringup" \
  "${TEMP_WS}/src/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -w /ws \
  vision60-autonomy:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install \
      --packages-select vision60_msgs vision60_bridge vision60_bringup
    source /ws/install/setup.bash
    colcon test --packages-select vision60_bridge
    colcon test-result --verbose
    ros2 launch vision60_bringup vision60_bridge_mock.launch.py \
      --show-args

    ros2 run vision60_bridge vision60_bridge \
      --ros-args \
      -p transport:=mock \
      -p allow_motion_output:=true >/tmp/bridge.log 2>&1 &
    bridge_pid=$!
    trap "kill -INT ${bridge_pid} 2>/dev/null || true" EXIT
    sleep 2
    timeout 12 ros2 run vision60_bridge bridge_integration_probe
  '

echo "VISION60_BRIDGE_TEST=PASS"
