#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMP_WS="$(mktemp -d /private/tmp/vision60_scene_model.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

mkdir -p "${TEMP_WS}/src"
cp -R \
  "${WORKSPACE_ROOT}/src/vision60_msgs" \
  "${WORKSPACE_ROOT}/src/scene_model_pipeline" \
  "${TEMP_WS}/src/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -w /ws \
  vision60-scene-model:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source /ws/install/setup.bash
    colcon test --event-handlers console_direct+
    colcon test-result --verbose

    ros2 run scene_model_pipeline generate_synthetic_scene \
      --output /ws/mock_input
    ros2 run scene_model_pipeline build_scene_model \
      --points /ws/mock_input/raw_points.ply \
      --image /ws/mock_input/camera.png \
      --calibration /ws/mock_input/calibration.json \
      --metadata /ws/mock_input/metadata.json \
      --output /ws/mock_products \
      --voxel-size 0.10 \
      --mesh-depth 6

    ros2 run scene_model_pipeline generate_synthetic_mcap \
      --output /ws/sequence_input \
      --frames 4
    ros2 run scene_model_pipeline build_cumulative_scene \
      --bag /ws/sequence_input/mission_sequence \
      --calibration /ws/sequence_input/calibration.json \
      --metadata /ws/sequence_input/metadata.json \
      --output /ws/sequence_products \
      --voxel-size 0.12 \
      --mesh-depth 6 \
      --max-frames 4

    ros2 run scene_model_pipeline generate_synthetic_overlay \
      --scene-manifest /ws/sequence_products/cumulative_manifest.json \
      --output /ws/mission_overlay.json
    ros2 run scene_model_pipeline build_command_view \
      --scene-dir /ws/sequence_products \
      --overlay /ws/mission_overlay.json \
      --output /ws/command_view_products

    python3 -c "
import json
from pathlib import Path
manifest = json.loads(Path(
    \"/ws/mock_products/manifest.json\"
).read_text())
assert manifest[\"products\"][\"colored_cloud\"][\"point_count\"] > 1000
assert manifest[\"products\"][\"voxel_map\"][\"occupied_count\"] > 0
assert manifest[\"products\"][\"voxel_map\"][\"free_count\"] > 0
assert manifest[\"products\"][\"voxel_map\"][\"unknown_count\"] > 0
assert manifest[\"products\"][\"mesh\"][\"triangle_count\"] > 0
print(\"SCENE_MODEL_ARTIFACT_CHECK=PASS\")
"

    python3 -c "
import json
from pathlib import Path
manifest = json.loads(Path(
    \"/ws/sequence_products/cumulative_manifest.json\"
).read_text())
assert manifest[\"frame_count\"] == 4
assert manifest[\"products\"][\"colored_cloud\"][\"point_count\"] > 8000
assert manifest[\"products\"][\"voxel_map\"][\"occupied_count\"] > 0
assert manifest[\"products\"][\"mesh\"][\"triangle_count\"] > 0
print(\"CUMULATIVE_SCENE_ARTIFACT_CHECK=PASS\")
"

    python3 -c "
import cv2
import json
from pathlib import Path
output = Path(\"/ws/command_view_products\")
manifest = json.loads(
    (output / \"command_view_manifest.json\").read_text()
)
image = cv2.imread(str(output / \"command_view.png\"))
assert image.shape == (720, 1280, 3)
assert manifest[\"counts\"][\"route_points\"] == 4
assert manifest[\"counts\"][\"recovery_route_points\"] == 3
assert manifest[\"counts\"][\"recovery_waypoints\"] == 1
assert manifest[\"counts\"][\"selected_recovery_waypoints\"] == 1
assert manifest[\"counts\"][\"communication_zones\"] == 1
assert manifest[\"counts\"][\"obstacles\"] == 1
assert manifest[\"counts\"][\"mission_events\"] == 2
assert manifest[\"counts\"][\"ros_markers\"] == 13
assert (
    manifest[\"communication_summary\"][\"final_channel\"]
    == \"mock_backup_wifi\"
)
assert (output / \"command_view_replay\").is_dir()
print(\"COMMAND_VIEW_ARTIFACT_CHECK=PASS\")
"
  '

echo "VISION60_SCENE_MODEL_TEST=PASS"
