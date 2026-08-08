#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_ROOT="${1:-${WORKSPACE_ROOT}/artifacts/scene_model_mock}"
TEMP_WS="$(mktemp -d /private/tmp/vision60_scene_artifact.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

mkdir -p "${TEMP_WS}/src" "${OUTPUT_ROOT}"
rm -rf \
  "${OUTPUT_ROOT}/sequence_input/mission_sequence" \
  "${OUTPUT_ROOT}/sequence_products" \
  "${OUTPUT_ROOT}/command_view"
rm -f "${OUTPUT_ROOT}/overlay_input/mission_overlay.json"
cp -R \
  "${WORKSPACE_ROOT}/src/scene_model_pipeline" \
  "${TEMP_WS}/src/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -v "${OUTPUT_ROOT}:/artifacts" \
  -w /ws \
  vision60-scene-model:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source /ws/install/setup.bash
    ros2 run scene_model_pipeline generate_synthetic_scene \
      --output /artifacts/input
    ros2 run scene_model_pipeline build_scene_model \
      --points /artifacts/input/raw_points.ply \
      --image /artifacts/input/camera.png \
      --calibration /artifacts/input/calibration.json \
      --metadata /artifacts/input/metadata.json \
      --output /artifacts/products \
      --voxel-size 0.10 \
      --mesh-depth 6
    ros2 run scene_model_pipeline generate_synthetic_mcap \
      --output /artifacts/sequence_input \
      --frames 4
    ros2 run scene_model_pipeline build_cumulative_scene \
      --bag /artifacts/sequence_input/mission_sequence \
      --calibration /artifacts/sequence_input/calibration.json \
      --metadata /artifacts/sequence_input/metadata.json \
      --output /artifacts/sequence_products \
      --voxel-size 0.12 \
      --mesh-depth 6 \
      --max-frames 4
    ros2 run scene_model_pipeline generate_synthetic_overlay \
      --scene-manifest \
        /artifacts/sequence_products/cumulative_manifest.json \
      --output /artifacts/overlay_input/mission_overlay.json
    ros2 run scene_model_pipeline build_command_view \
      --scene-dir /artifacts/sequence_products \
      --overlay /artifacts/overlay_input/mission_overlay.json \
      --output /artifacts/command_view
  '

echo "SCENE_MODEL_ARTIFACTS=${OUTPUT_ROOT}"
