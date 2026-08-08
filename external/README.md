# 외부 패키지 채택 기준

검증된 기능을 다시 만들지 않는다. 외부 패키지는 버전을 고정하고,
Vision 60 전용 코드에는 토픽·TF·SDK 어댑터와 임무 상태기계만 둔다.

## 현재 채택

| 기능 | 채택 패키지 | 통합 방식 |
|---|---|---|
| OS1-32 점군·IMU | Ouster 공식 `ouster-ros` 0.14.2 | `humble_core.repos` |
| 3D LiDAR–IMU SLAM | `koide3/glim` | Orin 전용 환경 |
| 상태추정 융합 | `robot_localization` Humble | `humble_core.repos` |
| 경로추종·속도완화 | Navigation2 Humble | Docker apt 패키지 |
| Frontier 자율탐색 | `robo-friends/m-explore-ros2` | 커밋 `326cf8a0b487`; 통신복구 안전 게이트 연동 |
| 장애물 안전정지 | Nav2 Collision Monitor | 전체 속도 체인 연결·Mock 점군 시험 완료 |
| 속도 명령 우선권 | `ros-teleop/twist_mux` 4.3.0 | 전체 속도 체인 연결·우선순위/잠금 Mock 완료 |
| ROS 토픽 지연·손실 주입 | `ros-tooling/topic_tools` 1.1.2 | LiDAR 0.8초 지연·100% 손실과 통신두절 복합시험 완료 |
| 컬러 점군·복셀·메쉬 | `isl-org/Open3D` 0.14.1 | Ubuntu 22.04 ARM64 별도 후처리 이미지·합성시험 완료 |
| 임무 원본 기록·재생 | `ros2/rosbag2` MCAP 0.15.16 | LiDAR·영상·GLIM pose MCAP 왕복 및 4프레임 누적시험 완료 |
| 3D 관제 오버레이 | ROS `visualization_msgs` 4.9.1 | 표준 `MarkerArray`와 `PointCloud2` MCAP 재생시험 완료 |
| 실시간 관제 연결 | Foxglove 공식 `foxglove_bridge` 3.4.2 | Humble ARM64 apt·읽기 전용·WebSocket 시험 완료 |
| 관제 PNG 생성 | OpenCV 4.5.4 | 화면 없는 Docker에서 CPU 렌더링·합성시험 완료 |
| 공개 LiDAR·컬러카메라 회귀 | Oxford 공식 `oxford_spires_dataset` | 고정 revision 실제 4프레임·보정·pose 시험 완료 |
| 대용량 ZIP 부분 추출 | `remotezip` 0.12.3 | 31.8GB archive에서 필요한 12.1MB만 추출 |
| Linux 네트워크 프로필 전환 | NetworkManager D-Bus/libnm | 실물 프로필 확인 후 transport 연결 |
| ROS 2 다중 인터페이스 | Eclipse Cyclone DDS | 인터페이스 우선순위 설정에 활용 |
| 2D/3D 인식 메시지 | ROS `vision_msgs` Humble | 검출기 교체 가능한 표준 입출력으로 적용 |
| 근접 중복 영상 감사 | `idealo/imagededup` 0.3.3-post2 | PHash 후보 생성, 자동 삭제 금지 |
| 화재·연기 학습/배포 | NVIDIA TAO 6.0 RT-DETR | COCO→ResNet-18→ONNX→TensorRT |

### Frontier 탐색 저장소 선정 근거

- `m-explore-ros2`는 ROS 2 Humble 이상과 Nav2 `NavigateToPose`를
  지원하고 `/explore/resume`으로 목표 취소와 재개를 제공한다.
- BSD 라이선스이며 커밋 `326cf8a0b487c34246bb8f3326afbcd69576dc60`으로
  고정했다. 외부 탐색 코드는 수정하지 않고 `exploration_safety_gate`가
  통신복구·로봇안전 상태를 탐색 정지·재개 명령으로 변환한다.
- 실행 빌드와 핵심 C++ 단위시험은 통과했다. upstream의 코드 서식 및
  package.xml lint 실패는 프로젝트 시험과 분리해 기록한다.

### 요구조자·위험물 인식 저장소 검토 결론

- NVIDIA 공식 `isaac_ros_object_detection`의 `release-3.2`는 Jetson Orin,
  JetPack 6.1/6.2, ROS 2 Humble 조합을 공식 지원하므로 실물 AI 추론의
  우선 후보로 정했다. Apache-2.0이며 `vision_msgs` 검출 결과를 쓴다.
- 이 저장소는 추론 엔진이며 재난 요구조자·위험물 학습 모델은 제공하지
  않는다. 학습·현장 검증 없이 완성형으로 채택할 수 없어 현재 소스에는
  복사하지 않았다.
- Ultralytics는 TensorRT 내보내기를 지원하지만 AGPL-3.0 또는 별도 상용
  라이선스 조건이 있어 기본 의존성으로 넣지 않았다.
- 셧다운 기간에는 `mission_perception`의 명시적 `simulation_color`
  backend로 센서→2D 검출→LiDAR 3D 위치→임무 사건→관제 표시를 검증한다.
  실물에서는 backend만 학습된 Isaac ROS 모델로 교체한다.
- 사람 기본 모델은 NVIDIA NGC `PeopleNet AMR deployable_v1.0`으로
  정했다. 로봇 높이 데이터가 포함된 공식 모델이지만 출력은 사람일 뿐
  요구조자 확정이 아니므로 항상 `victim_candidate`로 관제 확인을 거친다.
- 화재·연기는 D-Fire 커밋 `4bf9c31b18fa`와 공식 저장소가 연결한 Kaggle
  v1 배포본을 사용했다. 21,527장을 COCO로 변환하고 이미지 디코딩,
  SHA-256, 박스 범위와 분할 간 완전중복 검사를 통과했다. 이후 지각 해시
  감사에서 공식 분할 사이의 연속 장면 누출을 찾아 공식 분할은 학습에 쓰지
  않고, PHash+DHash 장면 component 전체가 한 split에만 들어가는 새 view를
  만들었다.
  범위 밖 박스 379개는 가시영역으로 자르고 면적 0 박스 18개는 제외했으며,
  원본 라벨은 수정하지 않고 품질 보고서에 기록했다.
  컬렉션은 CC0지만 원 저작권을 모두 보유하지 않는다는 고지가 있으므로
  공개 재배포 전 개별 출처 권리 검토가 필요하다.
- AIDER는 항공 장면 분류 데이터라 로봇 높이 객체 위치화 학습에는 쓰지
  않고 재난 배경 분류 참고용으로만 둔다.
- 검토 결과와 정확한 revision은 `external/perception_sources.json`,
  데이터 규격은 `docs/PERCEPTION_TRAINING.md`에 고정했다.

### Vision60 제어 저장소 추가 검토

- CMU Robomechanics Lab의 `quad-sdk`는 MIT 라이선스, 논문, 단위시험과
  Vision60 계열 하드웨어 추상화가 있는 신뢰 가능한 연구 레포다.
- 그러나 로봇의 저수준 토크·상태추정·보행제어까지 교체하는 구조이고 현재
  문서는 ROS 2 Jazzy 연구 코드임을 명시한다. 우리 환경은 ROS 2 Humble에서
  제조사 KRM의 검증된 보행제어를 유지하고 안전한 속도 명령만 전달한다.
- 따라서 전체 레포를 가져오지 않는다. KRM SDK 계약을 받기 전에는 현재
  `vision60_bridge`의 실물 transport를 fail-closed로 유지하는 것이 맞다.

### 학습·중복검사 저장소 추가 검토

- `imagededup`은 Apache-2.0, 500회 이상 커밋과 테스트를 가진 성숙한
  레포라 PHash 생성과 후보 검색에 채택했다. 해시 방식이 선택적으로 쓰는
  CNN·그래프 의존성을 즉시 불러오는 문제만 lazy import로 보정했다.
- D-Fire 21,527장은 pHash 거리 6 이하를 후보로만 기록한다. 유사 해시는
  같은 장면임을 확정하지 않으므로 자동 삭제나 자동 분할 이동은 하지 않는다.
- 화재·연기 학습은 NVIDIA 공식 TAO 6.0 RT-DETR ResNet-18을 채택했다.
  기존 COCO를 복사하지 않는 2클래스 view, 학습·평가·ONNX export·FP16
  TensorRT 설정과 Isaac ROS `Detection2DArray` 융합 어댑터까지 준비했다.

### 대체 통신 저장소 검토 결론

- NetworkManager는 저장된 연결 프로필 활성화와 장치 상태 확인을 공식
  D-Bus/libnm API로 제공하므로 실제 Orin 네트워크 제어 기반으로
  사용한다.
- Cyclone DDS는 여러 네트워크 인터페이스와 우선순위를 설정할 수 있어
  ROS 2 통신 경로 구성에 사용한다.
- 두 프로젝트 모두 신뢰 가능한 기반이지만, 복구점 이동·후보 채널
  순서·재시도·장애 원인 분류는 제공하지 않는다. 이 정책 부분만
  `communication_channel_manager`에서 직접 유지한다.
- 실제 NetworkManager transport는 Orin의 연결 프로필 이름과 대체
  통신장치가 확인되기 전까지 구현하지 않고 fail-closed로 둔다.
- 납품 자료에서 로봇 직접 유선망 `192.168.168.0/24`, Microhard, LTE,
  외부 Wi-Fi 구성을 확인했다. 장비별 비밀번호는 저장소에 넣지 않으며,
  실제 인터페이스와 프로필 이름은 셧다운 종료 첫날 기록한다.

### 속도 명령 우선권 패키지 교체 근거

기존 후보였던 `kobuki-base/cmd_vel_mux` 대신 `ros-teleop/twist_mux`를
사용한다.

- `twist_mux`는 공식 `humble` branch를 유지하고 ROS 2 Humble 릴리스가
  존재한다. 개발 이미지에서 `ros-humble-twist-mux 4.3.0-1jammy`와
  `ros-humble-twist-mux-msgs 3.0.1-1jammy`가 arm64로 조회된다.
  따라서 소스 빌드 없이 apt로 버전 고정이 가능하다.
- `std_msgs/Bool` lock을 우선순위와 함께 지원한다. 비상정지, 관제
  수동제어, 통신두절 복귀를 각각 lock과 입력 우선순위로 표현할 수 있어
  현재 안전 우선순위와 구조가 일치한다.
- lock 토픽은 일정 주기로 발행돼야 하며 발행이 끊기면 lock으로 간주한다.
  발행자 소실이 곧 차단이므로 fail-closed 원칙과 맞는다.

`full_system_mock.launch.py`에서 `vision60_bridge` 앞단에 연결했다.
자율주행 50, 관제 수동조작 100, 복귀 150, 안전 잠금 255로 동작하며
Mock에서 우선순위와 fail-closed 잠금을 검증했다.

### 센서 고장 주입 패키지 선정 근거

센서 토픽 지연·손실은 ROS Tooling 공식 `topic_tools`를 사용한다.
메시지 형식과 무관하게 동작하고 지연 시 원본 timestamp를 유지하므로
LiDAR freshness 게이트를 실제 조건과 같이 시험할 수 있다. 임무 시점에
맞춘 통신두절 시나리오와 PASS 판정만 프로젝트 코드로 유지한다.

### 다중 프레임 현장 모델 저장소 검토 결론

- MCAP 읽기·쓰기는 ROS 2 공식 `ros2/rosbag2`와
  `rosbag2_storage_mcap` 0.15.16을 그대로 사용한다.
- RTAB-Map ROS 2는 Humble과 RGB-D·3D LiDAR를 지원하지만 자체 SLAM,
  데이터베이스와 지도 생성 체계가 중심이다. 이미 선정한 GLIM 위치추정과
  역할이 겹치고 GLIM pose·임무 ID를 기준으로 한 파생파일 연결을 그대로
  제공하지 않아 현재 누적 후처리기로 채택하지 않았다.
- GLIM은 pose와 원본 3D 지도를 담당한다. LiDAR·영상 timestamp 최근접
  동기화, 카메라 투영, map 좌표 누적과 SHA-256 manifest만
  `scene_model_pipeline`의 얇은 어댑터로 직접 유지한다.

### 관제 시각화 저장소 검토 결론

- 경로·통신 이상구역·임무 사건은 ROS 공식 `visualization_msgs`의
  `MarkerArray`로 표현한다. 별도 사용자 메시지를 만들지 않아 RViz와
  Foxglove에서 같은 결과를 볼 수 있다.
- Foxglove Bridge는 고성능 ROS 2 실시간 연결에 적합하지만, 원본 모델과
  사건을 연결한 오프라인 PNG와 SHA-256 이력은 생성하지 않는다. 따라서
  라이브 전송 기반으로만 사용하고 산출물 생성은 얇은 어댑터로 유지한다.
- 공식 ROS 패키지 채널의 ARM64 `foxglove_bridge` 3.4.2를 채택했다. 공식
  Docker 이미지는 AMD64 전용이므로 Vision60의 Orin에는 쓰지 않는다.
  Bridge는 loopback에서만 열고 SSH 터널을 사용하며 client publish,
  서비스 호출, 파라미터 변경과 asset 조회를 모두 차단했다.
- 고정한 Open3D 0.14.1의 렌더러는 화면 없는 ARM64 Docker에서 X display를
  요구했다. 모델 생성은 Open3D를 계속 쓰고, 관제 PNG는 기존 이미지에
  포함된 OpenCV 4.5.4로 CPU 투영해 새 GUI 의존성을 추가하지 않았다.

### 공개 LiDAR·카메라 데이터셋 선정 결론

- KITTI는 센서·보정 구성이 충분하지만 현재 다운로드에 사용자 등록이
  필요해 자동 회귀시험 입력으로 사용하지 않았다.
- Oxford Robotics Institute의 Oxford Spires는 컬러 글로벌셔터 카메라
  3대, Hesai 64채널 LiDAR, IMU, 정밀 보정과 pose를 공개하고 ROS2 형식도
  제공한다. Vision60의 재난현장 보행 속도·건물 환경과 더 가깝다.
- 공식 도구 레포는 커밋 `b456e1e2f263a79c19b6ed4052390eba609011d4`,
  데이터는 revision `03f4382308333aa70c3253f12acd3fbf0c7c4a15`로
  고정했다.
- 전체 데이터는 1.3TB이고 선택 시퀀스의 이미지·LiDAR ZIP도 약 31.8GB다.
  MIT 라이선스 `remotezip` 0.12.3으로 동기 프레임 4쌍만 부분 추출해
  source 입력을 12.1MB로 제한했다.
- 데이터와 파생 결과는 CC-BY-NC-SA-4.0 비상업 학술 용도로만 사용하고
  원본 `LICENSE.md`, 출처 URL, revision과 모든 파일 해시를 보존한다.

## 직접 유지할 최소 코드

- Ghost Robotics/KRM SDK를 ROS 2로 바꾸는 `vision60_bridge`
- 통신품질 판단과 복구 상태기계
- 실제 주행경로·Communication Recovery Waypoint 기록
- 외부 패키지 사이의 토픽, QoS, TF, 시간 동기화 설정

## 분리 보류

- Nvblox는 현재 주 개발환경(Humble)과 섞지 않는다. JetPack/Isaac ROS
  최신 지원 기준이 Jazzy·JetPack 7.1·Jetson Thor이므로 호환 버전을
  확인한 별도 컨테이너에서만 평가한다.
- GLIM 실배포는 CUDA·GTSAM 의존성 때문에 Jetson Orin에서 구성한다.
  Mac ARM에서는 공식 AMD64 CPU 이미지를 에뮬레이션하여 공개 bag의
  토픽·TF·odometry 연결만 검증한다.
- Open3D는 주행 이미지에 넣지 않고 `vision60-scene-model:humble`
  후처리 이미지에서만 사용한다. 최신 0.19는 ARM64 Linux 공식
  바이너리 문제가 있어 Ubuntu Jammy ARM64 0.14.1을 고정했다.

## 가져오기

```bash
cd /ws
mkdir -p external_src
vcs import external_src < external/humble_core.repos
vcs import external_src < external/orin_slam.repos
```

`version`은 검토한 커밋으로 고정한다. 업데이트는 빌드·재생시험을 통과한
경우에만 별도 변경으로 반영한다.

Ouster `humble-devel`의 SDK 0.16.2 최신 커밋은 2026-07-29 현재
Humble ARM64 빌드에서 SDK VERSION 파싱 회귀가 재현됐다. 따라서 바로
이전 공식 0.14.2 커밋 `b2e8cc6`을 사용하며, 전체 드라이버 빌드와
ROS 실행파일·인터페이스 확인을 통과했다.
