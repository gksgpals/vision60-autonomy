#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MISSION_ROOT="${WORKSPACE_ROOT}/artifacts/full_system_calibrated_mock_replay"
SCENE_ROOT="${WORKSPACE_ROOT}/artifacts/full_system_calibrated_scene/command_view"
INTEGRATED_ROOT="${1:-${WORKSPACE_ROOT}/artifacts/integrated_calibrated_replay}"
TEMP_WS="$(mktemp -d /private/tmp/vision60_integrated_test.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

if [[ ! -f "${INTEGRATED_ROOT}/metadata.yaml" ]]; then
  echo "Integrated replay is missing: ${INTEGRATED_ROOT}" >&2
  exit 2
fi
mkdir -p \
  "${TEMP_WS}/src" \
  "${TEMP_WS}/inputs/mission" \
  "${TEMP_WS}/inputs/scene" \
  "${TEMP_WS}/inputs/integrated"
cp -R "${WORKSPACE_ROOT}/src/vision60_msgs" "${TEMP_WS}/src/"
cp -R "${WORKSPACE_ROOT}/src/scene_model_pipeline" "${TEMP_WS}/src/"
cp -R "${MISSION_ROOT}/." "${TEMP_WS}/inputs/mission/"
cp -R "${SCENE_ROOT}/command_view_replay/." \
  "${TEMP_WS}/inputs/scene/"
cp -R "${INTEGRATED_ROOT}/." "${TEMP_WS}/inputs/integrated/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -w /ws \
  vision60-scene-model:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source /ws/install/setup.bash
    colcon test --packages-select scene_model_pipeline \
      --event-handlers console_direct+
    colcon test-result --verbose
    ros2 run scene_model_pipeline validate_integrated_replay \
      --mission-bag /ws/inputs/mission \
      --scene-bag /ws/inputs/scene \
      --integrated-bag /ws/inputs/integrated
  '

python3 "${SCRIPT_DIR}/validate_same_source_artifacts.py" \
  --mission "${MISSION_ROOT}" \
  --scene "${WORKSPACE_ROOT}/artifacts/full_system_calibrated_scene" \
  --integrated "${INTEGRATED_ROOT}"

echo "INTEGRATED_OPERATOR_REPLAY_TEST=PASS"
