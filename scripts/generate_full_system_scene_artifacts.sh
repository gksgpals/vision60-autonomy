#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MISSION_ROOT="${WORKSPACE_ROOT}/artifacts/full_system_calibrated_mock_replay"
CONFIG_ROOT="${WORKSPACE_ROOT}/src/vision60_bringup/config/full_system_scene"
OUTPUT_ROOT="${1:-${WORKSPACE_ROOT}/artifacts/full_system_calibrated_scene}"
TEMP_WS="$(mktemp -d /private/tmp/vision60_same_source_scene.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

if [[ -e "${OUTPUT_ROOT}" ]]; then
  echo "Output already exists: ${OUTPUT_ROOT}" >&2
  exit 2
fi
if [[ ! -f "${MISSION_ROOT}/metadata.yaml" ]]; then
  echo "Source replay is missing: ${MISSION_ROOT}" >&2
  exit 2
fi
mkdir -p \
  "${TEMP_WS}/src" \
  "${TEMP_WS}/input/mission" \
  "${TEMP_WS}/input/config" \
  "${TEMP_WS}/output/overlay_input"
cp -R "${WORKSPACE_ROOT}/src/vision60_msgs" "${TEMP_WS}/src/"
cp -R "${WORKSPACE_ROOT}/src/scene_model_pipeline" "${TEMP_WS}/src/"
cp -R "${MISSION_ROOT}/." "${TEMP_WS}/input/mission/"
cp -R "${CONFIG_ROOT}/." "${TEMP_WS}/input/config/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -w /ws \
  vision60-scene-model:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source /ws/install/setup.bash
    ros2 run scene_model_pipeline build_cumulative_scene \
      --bag /ws/input/mission \
      --calibration-from-bag \
      --metadata /ws/input/config/metadata.json \
      --output /ws/output/sequence_products \
      --voxel-size 0.18 \
      --mesh-depth 6 \
      --max-frames 28 \
      --frame-interval-ms 3000 \
      --image-tolerance-ms 120 \
      --pose-tolerance-ms 100 \
      --pose-topic /vision60/odom
    ros2 run scene_model_pipeline build_mission_overlay \
      --scene-manifest \
        /ws/output/sequence_products/cumulative_manifest.json \
      --bag /ws/input/mission \
      --output /ws/output/overlay_input/mission_overlay.json
    ros2 run scene_model_pipeline build_command_view \
      --scene-dir /ws/output/sequence_products \
      --overlay /ws/output/overlay_input/mission_overlay.json \
      --output /ws/output/command_view
    cp /ws/input/config/metadata.json /ws/output/metadata.json
  '

mkdir -p "$(dirname "${OUTPUT_ROOT}")"
cp -R "${TEMP_WS}/output" "${OUTPUT_ROOT}"
echo "FULL_SYSTEM_SENSOR_SCENE=${OUTPUT_ROOT}"
