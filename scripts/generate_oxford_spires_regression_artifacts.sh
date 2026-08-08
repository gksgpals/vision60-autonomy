#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_ROOT="${1:-${WORKSPACE_ROOT}/artifacts/oxford_spires_regression}"
SOURCE_ROOT="${OUTPUT_ROOT}/source"
TEMP_WS="$(mktemp -d /private/tmp/vision60_oxford_artifact.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

if [[ ! -f "${SOURCE_ROOT}/sensor.yaml" ]]; then
  echo "Oxford sample is missing: ${SOURCE_ROOT}" >&2
  exit 2
fi

rm -rf \
  "${OUTPUT_ROOT}/converted" \
  "${OUTPUT_ROOT}/products" \
  "${OUTPUT_ROOT}/command_view"
mkdir -p "${TEMP_WS}/src" "${OUTPUT_ROOT}"
cp -R "${WORKSPACE_ROOT}/src/scene_model_pipeline" "${TEMP_WS}/src/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -v "${SOURCE_ROOT}:/oxford:ro" \
  -v "${OUTPUT_ROOT}:/artifacts" \
  -w /ws \
  vision60-scene-model:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source /ws/install/setup.bash
    ros2 run scene_model_pipeline convert_oxford_spires_sample \
      --input /oxford \
      --output /artifacts/converted
    ros2 run scene_model_pipeline build_cumulative_scene \
      --bag /artifacts/converted/oxford_spires_sequence \
      --calibration /artifacts/converted/calibration.json \
      --metadata /artifacts/converted/metadata.json \
      --output /artifacts/products \
      --voxel-size 0.25 \
      --mesh-depth 6 \
      --max-frames 4 \
      --image-tolerance-ms 25.0 \
      --pose-tolerance-ms 100.0
    ros2 run scene_model_pipeline build_command_view \
      --scene-dir /artifacts/products \
      --overlay /artifacts/converted/route_overlay.json \
      --output /artifacts/command_view
  '

echo "OXFORD_SPIRES_ARTIFACTS=${OUTPUT_ROOT}"
