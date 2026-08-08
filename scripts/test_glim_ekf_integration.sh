#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WORKSPACE_ROOT="${PROJECT_ROOT}/software/vision60_autonomy_ws"
BAG_PATH="${PROJECT_ROOT}/datasets.nosync/glim/os1_128_01_downsampled"
RESULT_PATH="${GLIM_INTEGRATION_RESULT_PATH:-${PROJECT_ROOT}/datasets.nosync/glim/integration_probe}"
IMAGE="vision60-glim-integration:humble-amd64"

if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
  echo "ERROR: ROS bag not found: ${BAG_PATH}" >&2
  echo "Run ./scripts/download_glim_sample.sh first." >&2
  exit 1
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  docker build \
    --platform linux/amd64 \
    -f "${WORKSPACE_ROOT}/docker/Dockerfile.glim_integration" \
    -t "${IMAGE}" \
    "${WORKSPACE_ROOT}"
fi

mkdir -p "${RESULT_PATH}"

docker run --rm \
  --platform linux/amd64 \
  -v "${BAG_PATH}:/bag:ro" \
  -v "${RESULT_PATH}:/results" \
  "${IMAGE}" \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    source /root/ros2_ws/install/setup.bash
    source /root/vision60_ws/install/setup.bash

    cleanup() {
      kill -INT "${bag_pid:-}" "${launch_pid:-}" "${tf_pid:-}" 2>/dev/null || true
    }
    trap cleanup EXIT

    # Sample-only transform. Replace with measured Vision60 sensor extrinsics.
    ros2 run tf2_ros static_transform_publisher \
      --x 0 --y 0 --z 0 \
      --roll 0 --pitch 0 --yaw 0 \
      --frame-id base_link \
      --child-frame-id os_imu >/results/static_tf.log 2>&1 &
    tf_pid=$!

    ros2 launch vision60_bringup glim_ouster.launch.py \
      use_ekf:=true >/results/launch.log 2>&1 &
    launch_pid=$!
    sleep 4

    timeout 90 ros2 topic echo /state/odometry --once \
      >/results/state_odometry.yaml 2>/results/state_probe.err &
    probe_pid=$!

    timeout 50 ros2 bag play /bag \
      --rate 1 \
      --topics /os_cloud_node/imu /os_cloud_node/points \
      --remap \
        /os_cloud_node/imu:=/ouster/imu \
        /os_cloud_node/points:=/ouster/points \
      --disable-keyboard-controls >/results/bag_play.log 2>&1 &
    bag_pid=$!

    wait "${probe_pid}"

    timeout 5 ros2 run tf2_ros tf2_echo map odom \
      >/results/tf_map_odom.txt 2>&1 || true
    timeout 5 ros2 run tf2_ros tf2_echo odom base_link \
      >/results/tf_odom_base_link.txt 2>&1 || true
  '

grep -q 'frame_id: odom' "${RESULT_PATH}/state_odometry.yaml"
grep -q 'child_frame_id: base_link' "${RESULT_PATH}/state_odometry.yaml"
grep -q 'Translation:' "${RESULT_PATH}/tf_map_odom.txt"
grep -q 'Translation:' "${RESULT_PATH}/tf_odom_base_link.txt"

echo "GLIM_EKF_INTEGRATION=PASS"
echo "Result: ${RESULT_PATH}"
