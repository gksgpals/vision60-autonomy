#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMP_WS="$(mktemp -d /private/tmp/vision60_operator.XXXXXX)"

cleanup() {
  rm -rf "${TEMP_WS}"
}
trap cleanup EXIT

mkdir -p "${TEMP_WS}/src"
cp -R "${WORKSPACE_ROOT}/src/vision60_bringup" "${TEMP_WS}/src/"

docker run --rm \
  -v "${TEMP_WS}:/ws" \
  -w /ws \
  vision60-autonomy:humble \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    colcon test --packages-select vision60_bringup
    colcon test-result --verbose
    source /ws/install/setup.bash

    ros2 launch vision60_bringup operator_bridge.launch.py \
      >/tmp/operator_bridge.log 2>&1 &
    launch_pid=$!
    trap "kill -INT ${launch_pid} 2>/dev/null || true" EXIT
    sleep 3

    ros2 node list | grep -Fx /foxglove_bridge
    ros2 param get /foxglove_bridge address | grep -F 127.0.0.1
    ros2 param get /foxglove_bridge port | grep -F 8765
    ros2 param get /foxglove_bridge capabilities \
      | grep -F connectionGraph

    python3 - <<"PY"
import socket

request = (
    "GET / HTTP/1.1\r\n"
    "Host: 127.0.0.1:8765\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
    "Sec-WebSocket-Version: 13\r\n"
    "Sec-WebSocket-Protocol: foxglove.sdk.v1\r\n\r\n"
)
with socket.create_connection(("127.0.0.1", 8765), timeout=5) as stream:
    stream.sendall(request.encode("ascii"))
    response = stream.recv(4096).decode("latin1")
if " 101 " not in response.split("\r\n", 1)[0]:
    raise SystemExit(f"WebSocket upgrade failed: {response!r}")
print("FOXGLOVE_WEBSOCKET_HANDSHAKE=PASS")
PY
  '

echo "VISION60_OPERATOR_VISUALIZATION_TEST=PASS"
