#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAG_DIR="${1:-${WORKSPACE_ROOT}/artifacts/full_system_calibrated_mock_replay}"
TEMP_WS="$(mktemp -d /private/tmp/vision60_replay_test.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
  echo "Replay bag is missing: ${BAG_DIR}" >&2
  exit 2
fi

mkdir -p "${TEMP_WS}/src"
cp -R "${WORKSPACE_ROOT}/src/vision60_msgs" "${TEMP_WS}/src/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -v "${BAG_DIR}:/bag" \
  -v "${SCRIPT_DIR}/validate_full_system_mock_replay.py:/validate.py:ro" \
  -w /ws \
  vision60-autonomy:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source /ws/install/setup.bash
    python3 /validate.py /bag
  '

echo "FULL_SYSTEM_MOCK_REPLAY_TEST=PASS"
