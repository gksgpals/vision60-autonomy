# Vision60 Autonomy Stack

ROS 2 Humble 기반 Vision 60 재난현장 자율주행 스택이다. 3D LiDAR·IMU·카메라를 이용한 위치추정과 환경 인식, Nav2 주행 안전, 실제 이동경로 기반 통신복구, 화재·연기 후보의 3차원 위치 산출, 임무 데이터 기록을 하나의 인터페이스로 구성한다.

실제 로봇 명령은 `vision60_bridge`만 통과한다. 보행 허용, 비상정지, 통신상태, 위치추정, LiDAR 수신시각 중 하나라도 안전조건을 만족하지 못하면 최종 속도는 0으로 제한된다.

## System Architecture

```text
Ouster LiDAR + IMU + RGB Camera
              │
              ├── Localization / TF ── map → odom → base_link → sensor
              ├── 3D Scene Model ───── point cloud → color → voxel → mesh
              └── Perception ───────── RT-DETR 2D detection + LiDAR 3D fusion
                                      │
Nav2 Plan → twist_mux → velocity smoother → collision monitor
                                      │
                           timestamp / safety gate
                                      │
                              vision60_bridge
                                      │
                                  Vision 60

Communication Monitor → Safe Stop → Recorded-route Return
                      → Channel Switch → Mission Sync
                      → Low-speed Reentry → Normal / Safe Stop
```

## Core Logic

### Motion safety

주행 명령은 다음 순서로 처리한다.

```text
Nav2 → twist_mux → Velocity Smoother → Collision Monitor
     → Timestamp Gate → Vision60 Bridge
```

`twist_mux`는 자율주행, 수동조작, 통신복구 명령의 우선순위를 관리한다. `Velocity Smoother`는 가속도와 감속도를 제한하고, `Collision Monitor`는 Ouster 점군과 로봇 footprint를 이용해 감속·정지 영역을 검사한다. `Timestamp Gate`는 오래된 LiDAR와 이동 명령을 차단한다. 마지막 `vision60_bridge`는 안전상태를 확인하고 ROS 속도 명령을 로봇 전송 형식으로 변환한다.

### Communication recovery

통신이 안정적인 위치와 실제 통과경로를 함께 기록한다. 링크 손실이 확인되면 신규 탐색을 중단하고 다음 상태기계를 실행한다.

```text
NORMAL → DEGRADED → STOPPING → RETURNING → CHANNEL_SWITCH
       → SYNCING → REENTRY_TEST → NORMAL
                               └→ SAFE_STOP
```

`STOPPING`에서 실제 정지를 확인한 후에만 복귀 주행을 허용한다. `RETURNING`은 계획경로가 아니라 로봇이 이미 통과한 경로를 역순으로 추종한다. 연결이 복구되지 않으면 대체 채널을 시험하고, 연결 후 온보드 임무 DB를 동기화한다. 재진입은 저장된 경로를 정방향으로 재구성하며 0.10 m/s로 제한한다. 재진입 중 링크가 다시 끊기면 경로를 취소하고 안전정지한다.

### Perception and 3D localization

RT-DETR 결과는 표준 `vision_msgs/Detection2DArray`로 입력된다. 클래스, 신뢰도, timestamp, 영상 경계를 검증한 뒤 같은 시각의 LiDAR 점군을 카메라 좌표로 투영해 검출 대상의 3차원 위치를 계산한다.

```text
RGB image → RT-DETR → 2D bounding box
                           + LiDAR + TF + timestamp
                           → map-frame 3D candidate
                           → MissionEvent + mission database
```

AI 결과는 확정 판정이 아닌 `fire_candidate`, `smoke_candidate`로 기록한다. 원본 영상, 점군, 위치, 신뢰도를 함께 보존해 관제자가 결과를 확인할 수 있게 한다.

### Scene model pipeline

원본과 파생 결과는 임무 ID, timestamp, 좌표계, SHA-256으로 연결한다.

```text
Raw LiDAR → Colored Point Cloud → Voxel Map → Mesh
          → Operator Image / 3D Command View
```

원본 점군은 변경하지 않고, 복셀은 점유·비점유·미관측 공간 관리에 사용한다. 메쉬와 렌더링 결과에는 이동경로, 통신 이상지역, 인식 후보를 중첩할 수 있다.

## ROS 2 Packages

| Package | Responsibility |
|---|---|
| `vision60_msgs` | 안전상태, 통신복구, 경로점, 사건 메시지와 서비스·액션 정의 |
| `vision60_bridge` | 로봇 명령 경계, 안전조건 검사, Mock/KRM 전송 인터페이스 |
| `comm_recovery_manager` | 통신 상태기계, 경로 복귀, 채널 전환, 저속 재진입 |
| `route_recorder` | 실제 이동경로와 통신복구 기준점 기록 |
| `mission_logger` | 임무 사건·센서 메타데이터 저장과 재연결 동기화 |
| `mission_perception` | 2D 검출 검증, LiDAR 융합, 3D 사건 생성 |
| `scene_model_pipeline` | 점군·복셀·메쉬·관제 결과 생성과 데이터 계보 관리 |
| `vision60_bringup` | Nav2, EKF, 안전 게이트, 전체 시스템 launch와 설정 |
| `vision60_mock` | 로봇·통신·센서 고장 Mock 및 통합 검증 probe |
| `vision60_simulation` | Gazebo Fortress Vision 60 디지털 트윈과 시험 harness |

## Build

### ROS 2 workspace

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Simulation image

```bash
docker build -f docker/Dockerfile.simulation \
  -t vision60-simulation:humble-fortress .
```

## Verification

전체 안전 체인 Mock 시험:

```bash
./scripts/test_full_system_mock.sh
```

Gazebo 센서·주행 시험:

```bash
./scripts/test_vision60_digital_twin_motion.sh
```

정적·동적 장애물 회피 시험:

```bash
./scripts/test_vision60_digital_twin_obstacle_avoidance.sh
./scripts/test_vision60_dynamic_obstacle_avoidance.sh
```

통신두절 복구와 전체 통합 시험:

```bash
./scripts/test_vision60_digital_twin_recovery.sh
./scripts/test_vision60_integrated_recovery.sh
```

Frontier 탐색·복구 시험:

```bash
./scripts/test_vision60_frontier_exploration.sh
```

카메라·LiDAR 인식 융합 시험:

```bash
./scripts/test_vision60_perception.sh
```

센서 지연·단절과 통신두절 결합 시험:

```bash
./scripts/test_sensor_comm_faults.sh
```

## RT-DETR Deployment

학습 데이터는 저장소에 포함하지 않는다. COCO 형식 데이터가 준비되면 NVIDIA TAO 컨테이너로 학습, 평가, ONNX 내보내기와 TensorRT 엔진 생성을 실행한다.

```bash
./scripts/run_tao_rtdetr.sh dry-run
./scripts/run_tao_rtdetr.sh train
./scripts/run_tao_rtdetr.sh evaluate
./scripts/run_tao_rtdetr.sh export
./scripts/run_tao_rtdetr.sh gen_trt_engine
```

학습 설정은 `training/tao_rtdetr/experiment.yaml`, Isaac ROS 연결 설정은 `src/vision60_bringup/config/mission_perception_rtdetr.yaml`에 있다.

## Hardware Boundary

Gazebo 모델은 Vision 60의 충돌 외형과 센서 계약을 재현하지만 제조사 관절·모터 동역학 모델은 포함하지 않는다. `KrmVision60Interface`는 실제 KRM SDK의 API와 상태 계약이 제공되기 전까지 fail-closed 상태를 유지한다. 실제 배포 전에는 로봇 SDK 바인딩, E-stop, 센서 외부보정, 통신 프로파일, 저속 보행, Jetson TensorRT 지연시간을 대상 하드웨어에서 검증해야 한다.

## References

- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Navigation2](https://docs.nav2.org/)
- [NVIDIA Isaac ROS RT-DETR](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/isaac_ros_rtdetr/index.html)
- [NVIDIA TAO RT-DETR](https://docs.nvidia.com/tao/tao-toolkit/latest/text/cv_finetuning/pytorch/object_detection/rt_detr.html)
- [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2)
- [GLIM](https://github.com/koide3/glim)
