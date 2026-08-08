#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MISSION_ROOT="${WORKSPACE_ROOT}/artifacts/full_system_calibrated_mock_replay"
SCENE_ROOT="${WORKSPACE_ROOT}/artifacts/full_system_calibrated_scene/sequence_products"
OUTPUT_ROOT="${1:-${WORKSPACE_ROOT}/artifacts/recovery_3d_command_view}"
TEMP_WS="$(mktemp -d /private/tmp/vision60_recovery_3d.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

if [[ -e "${OUTPUT_ROOT}" ]]; then
  echo "Output already exists: ${OUTPUT_ROOT}" >&2
  exit 2
fi
if [[ ! -f "${MISSION_ROOT}/metadata.yaml" ]]; then
  echo "Mission replay is missing: ${MISSION_ROOT}" >&2
  exit 2
fi
if [[ ! -f "${SCENE_ROOT}/cumulative_manifest.json" ]]; then
  echo "3D scene is missing: ${SCENE_ROOT}" >&2
  exit 2
fi

mkdir -p \
  "${TEMP_WS}/src" \
  "${TEMP_WS}/input/mission" \
  "${TEMP_WS}/input/scene" \
  "${TEMP_WS}/output/overlay"
cp -R "${WORKSPACE_ROOT}/src/vision60_msgs" "${TEMP_WS}/src/"
cp -R "${WORKSPACE_ROOT}/src/scene_model_pipeline" "${TEMP_WS}/src/"
cp -R "${MISSION_ROOT}/." "${TEMP_WS}/input/mission/"
cp -R "${SCENE_ROOT}/." "${TEMP_WS}/input/scene/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -w /ws \
  vision60-scene-model:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source /ws/install/setup.bash
    ros2 run scene_model_pipeline build_mission_overlay \
      --scene-manifest /ws/input/scene/cumulative_manifest.json \
      --bag /ws/input/mission \
      --output /ws/output/overlay/mission_overlay.json
    ros2 run scene_model_pipeline build_command_view \
      --scene-dir /ws/input/scene \
      --overlay /ws/output/overlay/mission_overlay.json \
      --output /ws/output/command_view
  '

mkdir -p "$(dirname "${OUTPUT_ROOT}")"
cp -R "${TEMP_WS}/output" "${OUTPUT_ROOT}"

ffmpeg -loglevel error -y \
  -i "${OUTPUT_ROOT}/command_view/recovery_3d_replay.mp4" \
  -c:v libx264 -preset medium -crf 22 -pix_fmt yuv420p \
  -movflags +faststart -an \
  "${OUTPUT_ROOT}/command_view/recovery_3d_replay_h264.mp4"
ffmpeg -loglevel error -y \
  -i "${OUTPUT_ROOT}/command_view/recovery_3d_replay_h264.mp4" \
  -vf "select='eq(n,20)+eq(n,70)+eq(n,115)+eq(n,165)',scale=640:360,tile=2x2" \
  -frames:v 1 -update 1 \
  "${OUTPUT_ROOT}/command_view/recovery_3d_replay_contact_sheet.png"

python3 - "${OUTPUT_ROOT}" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
manifest = json.loads(
    (root / 'command_view/command_view_manifest.json').read_text()
)
counts = manifest['counts']
summary = manifest['communication_summary']
assert counts['route_points'] >= 10
assert counts['recovery_route_points'] >= 2
assert counts['recovery_waypoints'] >= 1
assert counts['selected_recovery_waypoints'] >= 1
assert counts['communication_zones'] >= 1
assert counts['mesh_triangles'] >= 100
assert summary['final_channel'] == 'mock_backup_wifi'
assert 'RETURNING' in summary['state_sequence']
assert 'CHANNEL_SWITCH' in summary['state_sequence']
assert 'REENTRY_TEST' in summary['state_sequence']
assert (root / 'command_view/recovery_3d_replay_h264.mp4').stat().st_size \
    > 100_000
assert (
    root / 'command_view/recovery_3d_replay_contact_sheet.png'
).stat().st_size > 100_000
print('RECOVERY_3D_COMMAND_VIEW=PASS')
PY

echo "RECOVERY_3D_COMMAND_VIEW=${OUTPUT_ROOT}"
