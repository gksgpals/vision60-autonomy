#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMP_WS="$(mktemp -d /private/tmp/vision60_dynamic_obstacle.XXXXXX)"
ARTIFACT_DIR="${WORKSPACE_ROOT}/artifacts/digital_twin_dynamic_obstacle"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

mkdir -p "${TEMP_WS}/src" "${ARTIFACT_DIR}"
rm -f \
  "${ARTIFACT_DIR}/PASS" \
  "${ARTIFACT_DIR}/FAIL" \
  "${ARTIFACT_DIR}/digital_twin_dynamic_obstacle.mp4" \
  "${ARTIFACT_DIR}/dynamic_obstacle_report.json" \
  "${ARTIFACT_DIR}/dynamic_obstacle_result.png"
for package in vision60_msgs comm_recovery_manager vision60_simulation; do
  cp -R "${WORKSPACE_ROOT}/src/${package}" "${TEMP_WS}/src/"
done

docker run --rm --shm-size=512m \
  -v "${TEMP_WS}:/ws" \
  -v "${ARTIFACT_DIR}:/artifacts" \
  -w /ws \
  vision60-simulation:humble-fortress \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    colcon test --packages-select \
      vision60_msgs comm_recovery_manager vision60_simulation
    colcon test-result --verbose
    source /ws/install/setup.bash
    export IGN_GAZEBO_RESOURCE_PATH="/ws/install/vision60_simulation/share/vision60_simulation/models"
    ros2 launch vision60_simulation digital_twin_obstacle_avoidance.launch.py \
      dynamic_mode:=true output_dir:=/artifacts \
      >/tmp/digital_twin_dynamic_obstacle.log 2>&1 &
    launch_pid=$!
    trap "kill -INT ${launch_pid} 2>/dev/null || true" EXIT

    deadline=$((SECONDS + 105))
    while [[ ${SECONDS} -lt ${deadline} ]]; do
      if [[ -s /artifacts/PASS ]]; then
        tail -n 140 /tmp/digital_twin_dynamic_obstacle.log
        exit 0
      fi
      if [[ -s /artifacts/FAIL ]]; then
        tail -n 220 /tmp/digital_twin_dynamic_obstacle.log
        exit 1
      fi
      sleep 1
    done
    tail -n 260 /tmp/digital_twin_dynamic_obstacle.log
    exit 124
  '

test -s "${ARTIFACT_DIR}/digital_twin_dynamic_obstacle.mp4"
test -s "${ARTIFACT_DIR}/dynamic_obstacle_report.json"
echo "VISION60_DYNAMIC_OBSTACLE_AVOIDANCE=PASS artifact=${ARTIFACT_DIR}/digital_twin_dynamic_obstacle.mp4"
