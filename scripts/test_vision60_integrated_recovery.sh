#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMP_WS="$(mktemp -d /private/tmp/vision60_integrated_recovery.XXXXXX)"
TEMP_ARTIFACT_DIR="${TEMP_WS}/artifacts"
ARTIFACT_DIR="${WORKSPACE_ROOT}/artifacts/digital_twin_integrated_recovery"

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
  "${ARTIFACT_DIR}/digital_twin_integrated_recovery.mp4" \
  "${ARTIFACT_DIR}/integrated_recovery_report.json" \
  "${ARTIFACT_DIR}/integrated_recovery_result.png" \
  "${ARTIFACT_DIR}/integrated_recovery.log"
for package in \
  vision60_msgs route_recorder comm_recovery_manager mission_logger \
  vision60_mock vision60_simulation; do
  cp -R "${WORKSPACE_ROOT}/src/${package}" "${TEMP_WS}/src/"
done

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
    source /ws/install/setup.bash
    export IGN_GAZEBO_RESOURCE_PATH="/ws/install/vision60_simulation/share/vision60_simulation/models"
    ros2 launch vision60_simulation \
      digital_twin_integrated_recovery.launch.py \
      integrated_output_dir:=/artifacts \
      >/tmp/integrated_recovery.log 2>&1 &
    launch_pid=$!
    trap "kill -INT ${launch_pid} 2>/dev/null || true" EXIT

    deadline=$((SECONDS + 115))
    while [[ ${SECONDS} -lt ${deadline} ]]; do
      if [[ -s /artifacts/PASS ]]; then
        tail -n 180 /tmp/integrated_recovery.log
        sync
        sleep 2
        exit 0
      fi
      if [[ -s /artifacts/FAIL ]]; then
        tail -n 260 /tmp/integrated_recovery.log
        sync
        sleep 2
        exit 0
      fi
      sleep 1
    done
    tail -n 320 /tmp/integrated_recovery.log
    exit 124
  '

copy_artifacts
if [[ -s "${ARTIFACT_DIR}/FAIL" ]]; then
  exit 1
fi
test -s "${ARTIFACT_DIR}/digital_twin_integrated_recovery.mp4"
test -s "${ARTIFACT_DIR}/integrated_recovery_report.json"
if command -v ffmpeg >/dev/null 2>&1; then
  ffmpeg -loglevel error -y \
    -i "${ARTIFACT_DIR}/digital_twin_integrated_recovery.mp4" \
    -c:v libx264 -pix_fmt yuv420p -movflags +faststart -an \
    "${ARTIFACT_DIR}/digital_twin_integrated_recovery_h264.mp4"
  test -s "${ARTIFACT_DIR}/digital_twin_integrated_recovery_h264.mp4"
fi
echo "VISION60_INTEGRATED_RECOVERY=PASS artifact=${ARTIFACT_DIR}/digital_twin_integrated_recovery_h264.mp4"
