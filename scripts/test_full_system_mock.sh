#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMP_WS="$(mktemp -d /private/tmp/vision60_full_system.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

mkdir -p "${TEMP_WS}/src"
cp -R \
  "${WORKSPACE_ROOT}/src/vision60_msgs" \
  "${WORKSPACE_ROOT}/src/vision60_bridge" \
  "${WORKSPACE_ROOT}/src/vision60_mock" \
  "${WORKSPACE_ROOT}/src/route_recorder" \
  "${WORKSPACE_ROOT}/src/comm_recovery_manager" \
  "${WORKSPACE_ROOT}/src/mission_logger" \
  "${WORKSPACE_ROOT}/src/vision60_bringup" \
  "${TEMP_WS}/src/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -w /ws \
  vision60-autonomy:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source /ws/install/setup.bash
    colcon test
    colcon test-result --verbose

    ros2 launch vision60_bringup full_system_mock.launch.py \
      allow_motion_output:=true >/tmp/full_system.log 2>&1 &
    launch_pid=$!
    trap "kill -INT ${launch_pid} 2>/dev/null || true" EXIT
    sleep 8

    set +e
    timeout 150 ros2 run vision60_mock full_system_probe
    probe_status=$?
    set -e
    if [ ${probe_status} -ne 0 ]; then
      echo "--- launch log (tail) ---"
      tail -n 120 /tmp/full_system.log
      exit ${probe_status}
    fi
  '

echo "VISION60_FULL_SYSTEM_TEST=PASS"
