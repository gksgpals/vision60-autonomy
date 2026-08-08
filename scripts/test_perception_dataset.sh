#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMP_WS="$(mktemp -d /private/tmp/vision60_perception_data.XXXXXX)"
ARTIFACT_DIR="${WORKSPACE_ROOT}/artifacts/perception_dataset_validation"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

mkdir -p "${TEMP_WS}/src" "${TEMP_WS}/artifacts" "${ARTIFACT_DIR}"
cp -R "${WORKSPACE_ROOT}/src/vision60_msgs" "${TEMP_WS}/src/"
cp -R "${WORKSPACE_ROOT}/src/mission_perception" "${TEMP_WS}/src/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -w /ws \
  vision60-simulation:humble-fortress \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    colcon test --packages-select mission_perception
    colcon test-result --verbose
    source /ws/install/setup.bash
    ros2 run mission_perception generate_perception_dataset_fixture \
      --output /ws/artifacts/fixture
    ros2 run mission_perception validate_perception_dataset \
      --dataset /ws/artifacts/fixture \
      --report /ws/artifacts/dataset_validation_report.json
    ros2 run mission_perception generate_dfire_dataset_fixture \
      --output /ws/artifacts/dfire_source
    ros2 run mission_perception import_dfire_dataset \
      --source /ws/artifacts/dfire_source \
      --output /ws/artifacts/dfire_coco \
      --revision 4bf9c31b18fadcd44d5f0b6d66f82bc56fa5e328
    ros2 run mission_perception validate_perception_dataset \
      --dataset /ws/artifacts/dfire_coco \
      --report /ws/artifacts/dfire_validation_report.json
  '

cp -R "${TEMP_WS}/artifacts/." "${ARTIFACT_DIR}/"
test -s "${ARTIFACT_DIR}/dataset_validation_report.json"
test -s "${ARTIFACT_DIR}/dfire_validation_report.json"
echo "VISION60_PERCEPTION_DATASET=PASS report=${ARTIFACT_DIR}/dataset_validation_report.json"
