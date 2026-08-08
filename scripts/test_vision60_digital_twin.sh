#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMP_WS="$(mktemp -d /private/tmp/vision60_digital_twin.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

mkdir -p "${TEMP_WS}/src"
cp -R "${WORKSPACE_ROOT}/src/vision60_simulation" "${TEMP_WS}/src/"

docker run --rm --shm-size=512m \
  -v "${TEMP_WS}:/ws" \
  -w /ws \
  vision60-simulation:humble-fortress \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    colcon test --packages-select vision60_simulation
    colcon test-result --verbose
    source /ws/install/setup.bash

    export IGN_GAZEBO_RESOURCE_PATH="/ws/install/vision60_simulation/share/vision60_simulation/models"
    ros2 launch vision60_simulation digital_twin.launch.py headless:=true \
      >/tmp/digital_twin.log 2>&1 &
    launch_pid=$!
    trap "kill -INT ${launch_pid} 2>/dev/null || true" EXIT
    sleep 6

    set +e
    timeout 55 ros2 run vision60_simulation digital_twin_probe
    probe_status=$?
    set -e
    if [[ ${probe_status} -ne 0 ]]; then
      tail -n 160 /tmp/digital_twin.log
      exit ${probe_status}
    fi
  '

echo "VISION60_DIGITAL_TWIN_TEST=PASS"
