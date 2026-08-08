#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMP_WS="$(mktemp -d /private/tmp/vision60_digital_twin_motion.XXXXXX)"
ARTIFACT_DIR="${WORKSPACE_ROOT}/artifacts/digital_twin_motion_test"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

mkdir -p "${TEMP_WS}/src" "${ARTIFACT_DIR}"
cp -R "${WORKSPACE_ROOT}/src/vision60_simulation" "${TEMP_WS}/src/"

docker run --rm --shm-size=512m \
  -v "${TEMP_WS}:/ws" \
  -v "${ARTIFACT_DIR}:/artifacts" \
  -w /ws \
  vision60-simulation:humble-fortress \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    colcon test --packages-select vision60_simulation
    colcon test-result --verbose
    source /ws/install/setup.bash

    export IGN_GAZEBO_RESOURCE_PATH="/ws/install/vision60_simulation/share/vision60_simulation/models"
    ros2 launch vision60_simulation digital_twin.launch.py headless:=true \
      >/tmp/digital_twin_motion.log 2>&1 &
    launch_pid=$!
    trap "kill -INT ${launch_pid} 2>/dev/null || true" EXIT
    sleep 6

    set +e
    timeout 60 ros2 run vision60_simulation digital_twin_motion_probe \
      --ros-args -p output_dir:=/artifacts
    probe_status=$?
    set -e
    if [[ ${probe_status} -ne 0 ]]; then
      tail -n 180 /tmp/digital_twin_motion.log
      exit ${probe_status}
    fi
  '

test -s "${ARTIFACT_DIR}/digital_twin_test_result.png"
test -s "${ARTIFACT_DIR}/test_report.json"
test -s "${ARTIFACT_DIR}/digital_twin_drive_test.mp4"
echo "VISION60_DIGITAL_TWIN_MOTION_TEST=PASS artifact=${ARTIFACT_DIR}/digital_twin_test_result.png"
