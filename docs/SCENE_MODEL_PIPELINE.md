# 컬러 점군·복셀·메쉬 오프라인 파이프라인

`scene_model_pipeline`은 안전·주행 연산과 분리된 후처리 패키지다.
LiDAR 점군과 카메라 영상의 시간 차이를 먼저 검사한 뒤, 보정행렬로
카메라에 투영해 컬러 점군을 만들고 복셀 지도와 메쉬를 생성한다.

## 외부 패키지 선정

- Open3D 공식 레포의 점군 다운샘플링과 Poisson 메쉬 기능을 재사용한다.
- Ubuntu 22.04 ARM64가 제공하는 Open3D 0.14.1을 고정한다.
- 최신 Open3D 0.19 공식 바이너리는 ARM64 Linux 동적 링커 문제가
  명시돼 있어 사용하지 않는다.
- 최신 Isaac ROS Nvblox는 ROS 2 Jazzy·JetPack 7.1·Jetson Thor
  기준이므로 현재 Humble·AGX Orin 주행 환경에 섞지 않는다.
- 카메라–LiDAR 외부보정은 학교 개방 후
  `direct_visual_lidar_calibration`으로 측정한 값을 입력한다.

주행 이미지 `vision60-autonomy:humble`은 4.33GB로 유지하고,
Open3D는 별도 `vision60-scene-model:humble` 이미지에만 넣었다.

## 처리 순서

```text
원본 LiDAR PLY + 카메라 PNG + 보정 JSON + 임무 메타데이터
  → timestamp 허용오차 검사
  → 카메라 투영 및 z-buffer 컬러화
  → 컬러 점군 PLY
  → 0.1m 다운샘플 점군 + ray-tracing 복셀 지도
  → Poisson 메쉬 PLY
  → 원본·파생 파일 SHA-256 manifest
```

복셀 지도는 점유와 비점유 인덱스를 저장한다. 경계 안에서 두 배열에
없는 셀은 미관측 영역이다. 원본 파일은 수정하지 않고 모든 결과에
`mission_id`, `scene_id`, `source_id`, timestamp, pose와 해시를
연결한다.

## 다중 프레임 MCAP 누적

공식 `rosbag2_py`와 `rosbag2_storage_mcap` 0.15.16을 사용해 다음
토픽을 직접 읽는다.

```text
/ouster/points + /camera/image_raw + /slam/odom
  → timestamp 최근접 동기화
  → 프레임별 컬러 점군
  → GLIM pose로 map 좌표 변환
  → 누적 컬러 점군·복셀·메쉬
  → MCAP·보정·메타데이터·결과 SHA-256 manifest
```

영상은 50ms, pose는 100ms 이내 데이터만 사용하며 허용범위를 벗어난
프레임은 버린다. PointCloud2 frame과 Odometry child frame이 다르면
`pose_child_from_lidar` 외부보정이 없을 때 처리를 중단한다. 대용량 bag은
`--max-frames`까지만 읽고 이후 동기화 확인에 필요한 짧은 구간만 본다.

Mock MCAP은 0.2m 간격으로 이동하는 네 개 pose와 각 pose의 LiDAR·RGB
영상을 기록한 다음 파일을 다시 읽는다. 최종 결과는 map 좌표 컬러 점군
11,700점, 0.12m 복셀 점군 703점, 점유 604칸, 비점유 5,228칸,
미관측 14,628칸, 메쉬 정점 3,992개와 삼각형 7,701개다.

## 관제 이미지와 표준 ROS 오버레이

누적 메쉬, 실제 통과경로, 복귀 경로, 통신 복구 기준점, 통신 이상구역,
요구조자·위험물·장애물 후보를 동일한
`map` 좌표계에 중첩한다. 화면 없는 Docker에서도 실행되도록 OpenCV CPU
투영으로 1280×720 PNG와 상태 재생 MP4를 만들고, 같은 내용을 ROS 표준 `PointCloud2`와
`MarkerArray`로 MCAP에 저장한다. 이 재생 파일은 RViz와 Foxglove에서
별도 메시지 변환 없이 사용할 수 있다.

입력 `mission_overlay.json`의 `mission_id`, `scene_id`, 좌표계, 좌표값과
신뢰도 범위를 먼저 검사한다. PNG, MCAP, 누적 점군, 메쉬, 오버레이 JSON은
`command_view_manifest.json`에서 SHA-256으로 연결한다. Mock은 실제 경로,
역주행 경로, 선택된 복구 기준점, 최종 대체채널과 복구 상태 순서를 검증한다.

## Oxford Spires 실제 데이터 회귀시험

Oxford Robotics Institute의 공식 Oxford Spires Dataset에서 Keble College
시퀀스의 전방 컬러카메라·Hesai LiDAR·SLAM pose 4프레임을 사용한다. 공식
보정은 `OPENCV_FISHEYE` 모델로 읽으며, LiDAR와 카메라 시간 차이는
23.38~24.98ms다. 원본 31.8GB ZIP 전체를 받지 않고 `remotezip` 0.12.3으로
필요한 파일 12.1MB만 추출한다.

```bash
python3 -m venv /tmp/oxford-download
/tmp/oxford-download/bin/pip install \
  -r scripts/requirements-oxford-spires.txt
/tmp/oxford-download/bin/python \
  scripts/download_oxford_spires_sample.py \
  --output artifacts/oxford_spires_regression/source

./scripts/test_oxford_spires_regression.sh
./scripts/generate_oxford_spires_regression_artifacts.sh
```

회귀 결과는 컬러 점군 34,916점, 0.25m 복셀 9,594점, 점유 9,235칸,
비점유 304,026칸, 메쉬 정점 3,894개와 삼각형 7,588개다. 첫 프레임의
LiDAR 8,784점을 카메라에 다시 투영한 `projection_overlay.png`로 건물·지면
경계 정합을 육안 확인하고, 모든 source와 파생파일을 SHA-256으로 연결한다.

## 빌드와 시험

```bash
docker build \
  -t vision60-scene-model:humble \
  -f docker/Dockerfile.scene_model .

./scripts/test_scene_model_pipeline.sh
./scripts/generate_scene_model_mock_artifacts.sh
```

2026-07-31 ARM64 Mock 결과는 컬러 점군 3,996점, 0.1m 복셀 점군
1,543점, 점유 1,178칸, 비점유 12,670칸, 미관측 38,512칸,
메쉬 정점 4,222개와 삼각형 8,165개다. 전체 시험 22개는 실패 없이
통과했다.

전체 통신복구 Mock은 LiDAR·카메라·로봇 위치를 한 MCAP에 기록한다.
3초 간격으로 전 구간을 샘플링해 유효 24프레임에서 컬러 점군 17,671점,
점유 복셀 1,210칸과 메쉬 삼각형 2,382개를 생성했다. 카메라 시야에 점이
없는 3프레임은 오류를 숨기지 않고 manifest의 제외 사유로 남겼다.

생성 결과:

- `artifacts/scene_model_mock/input`: 변경되지 않는 원본과 보정·메타데이터
- `artifacts/scene_model_mock/products/colored_cloud.ply`
- `artifacts/scene_model_mock/products/voxel_cloud.ply`
- `artifacts/scene_model_mock/products/voxel_map.npz`
- `artifacts/scene_model_mock/products/scene_mesh.ply`
- `artifacts/scene_model_mock/products/manifest.json`
- `artifacts/scene_model_mock/sequence_input/mission_sequence`: Mock MCAP
- `artifacts/scene_model_mock/sequence_products`: 누적 점군·복셀·메쉬·manifest
- `artifacts/scene_model_mock/overlay_input/mission_overlay.json`: Mock 오버레이
- `artifacts/scene_model_mock/command_view/command_view.png`: 관제 이미지
- `artifacts/scene_model_mock/command_view/command_view_replay`: ROS MCAP 재생
- `artifacts/scene_model_mock/command_view/command_view_manifest.json`: 생성 이력
- `artifacts/oxford_spires_regression/source`: 라이선스 포함 실제 입력 4쌍
- `artifacts/oxford_spires_regression/converted/projection_overlay.png`: 정합 확인
- `artifacts/oxford_spires_regression/products`: 실제 누적 3D 결과
- `artifacts/oxford_spires_regression/command_view`: 실제 데이터 관제 이미지
- `artifacts/full_system_calibrated_mock_replay`: 복구·센서·보정·TF 동일 원본
- `artifacts/full_system_calibrated_scene`: MCAP 내부 보정으로 생성한 3D 장면
- `artifacts/integrated_calibrated_replay`: 원래 시각을 보존한 통합 MCAP
- `artifacts/recovery_3d_command_view`: 실제 MCAP 기반 복구 3D 관제 이미지·영상

RViz 관제 설정, Foxglove 패널 토픽 계약과 읽기 전용 실시간 Bridge를
연결했고 통신복구 전체 Mock도 MCAP으로 기록했다. `CameraInfo`와
`/tf_static`을 MCAP에 함께 저장하므로 파생 장면이 외부 보정 파일에 의존하지
않는다. 학교 개방 후에는 가상값을 실제 장착 카메라의 측정값으로 교체한다.
