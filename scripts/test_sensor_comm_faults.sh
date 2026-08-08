#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMP_WS="$(mktemp -d /private/tmp/vision60_sensor_fault.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

mkdir -p "${TEMP_WS}/src"
cp -R \
  "${WORKSPACE_ROOT}/src/vision60_msgs" \
  "${WORKSPACE_ROOT}/src/vision60_mock" \
  "${WORKSPACE_ROOT}/src/comm_recovery_manager" \
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
  '

for mode in delay drop; do
  docker run --rm \
    -e FAULT_MODE="${mode}" \
    -v "${TEMP_WS}:/ws" \
    -w /ws \
    vision60-autonomy:humble \
    bash -lc '
      set -e
      source /opt/ros/humble/setup.bash
      source /ws/install/setup.bash
      ros2 launch vision60_bringup sensor_comm_fault_mock.launch.py \
        fault_mode:=${FAULT_MODE} >/tmp/sensor_fault.log 2>&1 &
      launch_pid=$!
      trap "kill -TERM ${launch_pid} 2>/dev/null || true" EXIT
      sleep 2

      set +e
      timeout 20 ros2 run vision60_mock sensor_comm_fault_probe \
        --ros-args -p expected_mode:=${FAULT_MODE}
      probe_status=$?
      set -e

      if [ ${probe_status} -ne 0 ]; then
        tail -n 120 /tmp/sensor_fault.log
        exit ${probe_status}
      fi
    '
done

echo "VISION60_SENSOR_COMM_FAULT_TEST=PASS"
