#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMP_WS="$(mktemp -d /private/tmp/vision60_frontier.XXXXXX)"
TEMP_ARTIFACT_DIR="${TEMP_WS}/artifacts"
ARTIFACT_DIR="${WORKSPACE_ROOT}/artifacts/digital_twin_frontier_exploration"

copy_artifacts() {
  if [[ -d "${TEMP_ARTIFACT_DIR}" ]]; then
    cp -R "${TEMP_ARTIFACT_DIR}/." "${ARTIFACT_DIR}/"
  fi
}

cleanup() {
  copy_artifacts
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

mkdir -p "${TEMP_WS}/src" "${TEMP_ARTIFACT_DIR}" "${ARTIFACT_DIR}"
rm -f \
  "${ARTIFACT_DIR}/PASS" \
  "${ARTIFACT_DIR}/FAIL" \
  "${ARTIFACT_DIR}/digital_twin_frontier_exploration.mp4" \
  "${ARTIFACT_DIR}/frontier_exploration_report.json" \
  "${ARTIFACT_DIR}/frontier_exploration_result.png" \
  "${ARTIFACT_DIR}/frontier_exploration.log"

for package in \
  vision60_msgs route_recorder comm_recovery_manager mission_logger \
  vision60_mock vision60_simulation; do
  cp -R "${WORKSPACE_ROOT}/src/${package}" "${TEMP_WS}/src/"
done
cp -R \
  "${WORKSPACE_ROOT}/external_src/m-explore-ros2/explore" \
  "${TEMP_WS}/src/"
cp -R \
  "${WORKSPACE_ROOT}/external_src/m-explore-ros2/explore_lite_msgs" \
  "${TEMP_WS}/src/"

docker run --rm --shm-size=512m \
  -v "${TEMP_WS}:/ws" \
  -v "${TEMP_ARTIFACT_DIR}:/artifacts" \
  -w /ws \
  vision60-simulation:humble-fortress \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    colcon test --packages-select \
      vision60_msgs route_recorder comm_recovery_manager mission_logger \
      vision60_mock vision60_simulation
    colcon test-result --verbose
    ctest --test-dir /ws/build/explore_lite \
      --output-on-failure -R '^test_explore$'
    source /ws/install/setup.bash
    export IGN_GAZEBO_RESOURCE_PATH="/ws/install/vision60_simulation/share/vision60_simulation/models"
    ros2 launch vision60_simulation \
      digital_twin_frontier_exploration.launch.py \
      output_dir:=/artifacts \
      >/tmp/frontier_exploration.log 2>&1 &
    launch_pid=$!
    trap "kill -INT ${launch_pid} 2>/dev/null || true" EXIT

    deadline=$((SECONDS + 150))
    while [[ ${SECONDS} -lt ${deadline} ]]; do
      if [[ -s /artifacts/PASS ]]; then
        tail -n 220 /tmp/frontier_exploration.log
        cp /tmp/frontier_exploration.log /artifacts/frontier_exploration.log
        sync
        sleep 2
        exit 0
      fi
      if [[ -s /artifacts/FAIL ]]; then
        tail -n 320 /tmp/frontier_exploration.log
        cp /tmp/frontier_exploration.log /artifacts/frontier_exploration.log
        sync
        sleep 2
        exit 0
      fi
      sleep 1
    done
    tail -n 360 /tmp/frontier_exploration.log
    cp /tmp/frontier_exploration.log /artifacts/frontier_exploration.log
    exit 124
  '

copy_artifacts
if [[ -s "${ARTIFACT_DIR}/FAIL" ]]; then
  exit 1
fi
test -s "${ARTIFACT_DIR}/digital_twin_frontier_exploration.mp4"
test -s "${ARTIFACT_DIR}/frontier_exploration_report.json"
echo "VISION60_FRONTIER_EXPLORATION=PASS artifact=${ARTIFACT_DIR}/digital_twin_frontier_exploration.mp4"
