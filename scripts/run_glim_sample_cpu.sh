#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

DATASET_ROOT="${GLIM_DATASET_ROOT:-${PROJECT_ROOT}/datasets.nosync/glim}"
BAG_PATH="${DATASET_ROOT}/os1_128_01_downsampled"
CONFIG_PATH="${PROJECT_ROOT}/software/vision60_autonomy_ws/src/vision60_bringup/config/glim_cpu_sample"
RESULT_PATH="${GLIM_RESULT_PATH:-${DATASET_ROOT}/results_cpu_auto}"

if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
  echo "ERROR: ROS bag not found: ${BAG_PATH}" >&2
  echo "Run ./scripts/download_glim_sample.sh first." >&2
  exit 1
fi

mkdir -p "${RESULT_PATH}"

docker run --rm \
  --platform linux/amd64 \
  -v "${BAG_PATH}:/bag:ro" \
  -v "${CONFIG_PATH}:/glim-config:ro" \
  -v "${RESULT_PATH}:/tmp/dump" \
  koide3/glim_ros2:humble \
  bash -lc '
    source /opt/ros/humble/setup.bash
    source /root/ros2_ws/install/setup.bash
    ros2 run glim_ros glim_rosbag /bag \
      --ros-args \
      -p config_path:=/glim-config \
      -p auto_quit:=true \
      -p dump_path:=/tmp/dump
  '

if [[ ! -s "${RESULT_PATH}/traj_lidar.txt" ]]; then
  echo "ERROR: GLIM did not create traj_lidar.txt" >&2
  exit 1
fi

echo "GLIM CPU SAMPLE PASS"
echo "Result: ${RESULT_PATH}"
echo "LiDAR poses: $(wc -l < "${RESULT_PATH}/traj_lidar.txt" | tr -d ' ')"
