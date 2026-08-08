#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_DIR="${1:-${WORKSPACE_ROOT}/artifacts/full_system_calibrated_mock_replay}"
OUTPUT_PARENT="$(dirname "${OUTPUT_DIR}")"
OUTPUT_NAME="$(basename "${OUTPUT_DIR}")"
TEMP_WS="$(mktemp -d /private/tmp/vision60_replay.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

if [[ -e "${OUTPUT_DIR}" ]]; then
  echo "Output already exists: ${OUTPUT_DIR}" >&2
  exit 2
fi

mkdir -p "${TEMP_WS}/src" "${OUTPUT_PARENT}"
cp -R \
  "${WORKSPACE_ROOT}/src/vision60_msgs" \
  "${WORKSPACE_ROOT}/src/vision60_bridge" \
  "${WORKSPACE_ROOT}/src/vision60_mock" \
  "${WORKSPACE_ROOT}/src/route_recorder" \
  "${WORKSPACE_ROOT}/src/comm_recovery_manager" \
  "${WORKSPACE_ROOT}/src/mission_logger" \
  "${WORKSPACE_ROOT}/src/vision60_bringup" \
  "${TEMP_WS}/src/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -v "${OUTPUT_PARENT}:/artifacts" \
  -v "${SCRIPT_DIR}/validate_full_system_mock_replay.py:/validate.py:ro" \
  -w /ws \
  vision60-autonomy:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source /ws/install/setup.bash

    ros2 bag record --storage mcap \
      --output "/artifacts/'"${OUTPUT_NAME}"'" \
      /communication/state \
      /communication/recovery_status \
      /communication/recovery_event \
      /mission/event \
      /mission/sync_status \
      /mission/recorded_path \
      /mission/recovery_path \
      /mission/recovery_waypoint \
      /vision60/odom \
      /vision60/safety_state \
      /safety/collision_monitor_state \
      /vision60/mock_obstacle_phase \
      /cmd_vel_safe \
      /ouster/points \
      /camera/image_raw \
      /camera/camera_info \
      /tf \
      /tf_static \
      >/tmp/record.log 2>&1 &
    record_pid=$!

    ros2 launch vision60_bringup full_system_mock.launch.py \
      allow_motion_output:=true >/tmp/full_system.log 2>&1 &
    launch_pid=$!

    stop_processes() {
      kill -INT ${record_pid} 2>/dev/null || true
      wait ${record_pid} 2>/dev/null || true
      kill -TERM ${launch_pid} 2>/dev/null || true
      wait ${launch_pid} 2>/dev/null || true
    }
    trap stop_processes EXIT
    sleep 8

    set +e
    timeout 150 ros2 run vision60_mock full_system_probe
    probe_status=$?
    set -e
    if [[ ${probe_status} -ne 0 ]]; then
      tail -n 120 /tmp/full_system.log
      exit ${probe_status}
    fi

    kill -INT ${record_pid}
    wait ${record_pid}
    kill -TERM ${launch_pid} 2>/dev/null || true
    wait ${launch_pid} 2>/dev/null || true
    trap - EXIT

    python3 /validate.py "/artifacts/'"${OUTPUT_NAME}"'"
    ros2 bag info "/artifacts/'"${OUTPUT_NAME}"'"
  '

echo "FULL_SYSTEM_MOCK_REPLAY=${OUTPUT_DIR}"
