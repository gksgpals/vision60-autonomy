#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SOURCE_ROOT="${1:-${WORKSPACE_ROOT}/artifacts/oxford_spires_regression/source}"
TEMP_WS="$(mktemp -d /private/tmp/vision60_oxford_test.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

if [[ ! -f "${SOURCE_ROOT}/sensor.yaml" ]]; then
  echo "Oxford sample is missing: ${SOURCE_ROOT}" >&2
  exit 2
fi

mkdir -p "${TEMP_WS}/src"
cp -R "${WORKSPACE_ROOT}/src/scene_model_pipeline" "${TEMP_WS}/src/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -v "${SOURCE_ROOT}:/oxford:ro" \
  -w /ws \
  vision60-scene-model:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source /ws/install/setup.bash
    colcon test --event-handlers console_direct+
    colcon test-result --verbose
    ros2 run scene_model_pipeline convert_oxford_spires_sample \
      --input /oxford \
      --output /ws/converted
    ros2 run scene_model_pipeline build_cumulative_scene \
      --bag /ws/converted/oxford_spires_sequence \
      --calibration /ws/converted/calibration.json \
      --metadata /ws/converted/metadata.json \
      --output /ws/products \
      --voxel-size 0.25 \
      --mesh-depth 6 \
      --max-frames 4 \
      --image-tolerance-ms 25.0 \
      --pose-tolerance-ms 100.0
    ros2 run scene_model_pipeline build_command_view \
      --scene-dir /ws/products \
      --overlay /ws/converted/route_overlay.json \
      --output /ws/command_view
    python3 -c "
import cv2
import json
import open3d as o3d
from pathlib import Path
converted = Path(\"/ws/converted\")
products = Path(\"/ws/products\")
source = json.loads((converted / \"source_manifest.json\").read_text())
scene = json.loads((products / \"cumulative_manifest.json\").read_text())
view = json.loads(
    (Path(\"/ws/command_view\") / \"command_view_manifest.json\").read_text()
)
cloud = o3d.io.read_point_cloud(
    str(products / \"cumulative_colored_cloud.ply\")
)
image = cv2.imread(\"/ws/command_view/command_view.png\")
assert len(source[\"selected_frames\"]) == 4
assert max(x[\"image_delta_ms\"] for x in source[\"selected_frames\"]) <= 25.0
assert source[\"products\"][\"projection_overlay\"][\"projected_point_count\"] > 1000
assert scene[\"frame_count\"] == 4
assert scene[\"products\"][\"colored_cloud\"][\"point_count\"] > 1000
assert scene[\"products\"][\"mesh\"][\"triangle_count\"] > 0
assert len(cloud.colors) == len(cloud.points)
assert view[\"counts\"][\"route_points\"] == 4
assert image.shape == (720, 1280, 3)
print(\"OXFORD_SPIRES_REGRESSION_CHECK=PASS\")
"
  '

echo "VISION60_OXFORD_SPIRES_TEST=PASS"
