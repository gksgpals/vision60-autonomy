#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MISSION_ROOT="${WORKSPACE_ROOT}/artifacts/full_system_calibrated_mock_replay"
SCENE_ROOT="${WORKSPACE_ROOT}/artifacts/full_system_calibrated_scene/command_view"
OUTPUT_ROOT="${1:-${WORKSPACE_ROOT}/artifacts/integrated_calibrated_replay}"
TEMP_WS="$(mktemp -d /private/tmp/vision60_integrated_replay.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

if [[ -e "${OUTPUT_ROOT}" ]]; then
  echo "Output already exists: ${OUTPUT_ROOT}" >&2
  exit 2
fi
mkdir -p \
  "${TEMP_WS}/src" \
  "${TEMP_WS}/generated" \
  "${TEMP_WS}/inputs/mission" \
  "${TEMP_WS}/inputs/scene"
cp -R "${WORKSPACE_ROOT}/src/vision60_msgs" "${TEMP_WS}/src/"
cp -R "${WORKSPACE_ROOT}/src/scene_model_pipeline" "${TEMP_WS}/src/"
cp -R "${MISSION_ROOT}/." "${TEMP_WS}/inputs/mission/"
cp -R "${SCENE_ROOT}/." "${TEMP_WS}/inputs/scene/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -w /ws \
  vision60-scene-model:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source /ws/install/setup.bash
    ros2 run scene_model_pipeline build_integrated_replay \
      --mission-bag /ws/inputs/mission \
      --mission-manifest /ws/inputs/mission/replay_manifest.json \
      --scene-bag /ws/inputs/scene/command_view_replay \
      --scene-manifest /ws/inputs/scene/command_view_manifest.json \
      --preserve-scene-time \
      --output /ws/generated/integrated_operator_replay
  '

mkdir -p "$(dirname "${OUTPUT_ROOT}")"
cp -R "${TEMP_WS}/generated/integrated_operator_replay" "${OUTPUT_ROOT}"
echo "INTEGRATED_OPERATOR_REPLAY=${OUTPUT_ROOT}"
