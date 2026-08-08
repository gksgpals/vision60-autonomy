# Vision 60 Humble 개발 이미지

빌드:

```bash
docker build \
  -f docker/Dockerfile.humble \
  -t vision60-autonomy:humble .
```

포함 항목:

- ROS 2 Humble
- Navigation2
- Nav2 Bringup
- TF 진단 도구
- ARM64 공식 Foxglove Bridge
- rosbag2 MCAP 저장 플러그인 0.15.16

관제 연결은 `ros2 launch vision60_bringup operator_bridge.launch.py`로
실행한다. 기본값은 SSH 터널용 loopback `127.0.0.1:8765`이며 읽기 전용이다.
자세한 화면 구성은 `docs/OPERATOR_VISUALIZATION.md`를 따른다.

## GLIM 통합 검증 이미지

Mac ARM에서 공식 AMD64 GLIM 이미지와 `robot_localization` 연결을
검증할 때만 사용한다.

```bash
docker build \
  --platform linux/amd64 \
  -f docker/Dockerfile.glim_integration \
  -t vision60-glim-integration:humble-amd64 .
```

실제 Orin 배포본은 ARM64 환경에서 GLIM을 소스 빌드하여 구성한다.

## 3D 현장 모델 후처리 이미지

Open3D는 주행 안전 노드와 분리한 ARM64 후처리 이미지에서만 사용한다.

```bash
docker build \
  -f docker/Dockerfile.scene_model \
  -t vision60-scene-model:humble .
```

Ubuntu 22.04 ARM64 배포본인 Open3D 0.14.1과 Humble
`rosbag2_storage_mcap` 0.15.16을 정확히 고정한다.
관제 PNG는 이미지에 이미 포함된 OpenCV 4.5.4의 CPU 경로로 생성하며,
오버레이는 ROS `visualization_msgs/MarkerArray` MCAP으로 함께 저장한다.
기본 주행 이미지는 4.33GB, 후처리 이미지는 4.47GB다.

## Gazebo Fortress 물리 디지털 트윈

```bash
docker build \
  -f docker/Dockerfile.simulation \
  -t vision60-simulation:humble-fortress .
```

ROS 2 Humble이 정식으로 지원하는 Gazebo Fortress와 `ros_gz`를 고정하며,
headless 환경에서 IMU·3D LiDAR·카메라·보정정보를 검증한다.
