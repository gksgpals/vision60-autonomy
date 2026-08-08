# 관제 화면 연결

관제 연결은 ROS 메시지를 그대로 전달하는 공식 `foxglove_bridge`를 쓴다.
Vision60/Jetson은 ARM64이므로 AMD64 전용 Foxglove Docker 이미지를 쓰지 않고,
ROS Humble ARM64 패키지를 기본 이미지에 설치했다.

## 실시간 연결

로봇에서 다음 노드만 추가로 실행한다.

```bash
ros2 launch vision60_bringup operator_bridge.launch.py
```

기본 주소는 `127.0.0.1:8765`다. 관제 PC에서 SSH 터널을 열고 Foxglove의
`Foxglove WebSocket` 주소를 `ws://127.0.0.1:8765`로 설정한다.

```bash
ssh -N -L 8765:127.0.0.1:8765 ROBOT_USER@ROBOT_IP
```

브리지는 읽기 전용이다. 점군, 카메라, 실제 경로, 복귀 경로, 통신상태,
안전상태만 전달하고 화면에서 로봇 명령·서비스·파라미터 변경은 허용하지
않는다. 허용 토픽의 기준 파일은 `config/operator_topics.yaml`이다.

## 화면 구성

Foxglove에는 3D, Image, Plot, Raw Messages 패널을 만든다. 3D 패널에는
`/mission/scene_cloud`, `/mission/scene_markers`, `/mission/scene_mesh`,
`/mission/voxel_markers`, `/mission/recorded_path`, `/mission/recovery_path`를
켜고 표시 좌표계를 `map`으로 둔다. Image는
`/camera/image_raw`, Plot은 `/communication/state`의 신호세기·SNR·손실률·
지연을 표시한다. Raw Messages에는 `/communication/recovery_status`,
`/mission/event`, `/vision60/safety_state`를 표시한다.

Foxglove 2.58.0에서 실제 생성한 Mock 관제 레이아웃은 다음 파일이다.

```text
src/vision60_bringup/config/foxglove/vision60_operator_layout.json
```

Foxglove의 `Layouts > Import from file...`에서 이 파일을 불러온다. 레이아웃은
3D 컬러점군·메쉬·복셀·기록·복귀 경로, 통신품질 그래프, 안전 주행 그래프,
복구·통신·동기화 상태 전이, Mock 장애물 단계와 임무 사건을 표시한다. 전체 시스템 Mock의 좌표계는
`odom -> base_link`이므로 3D 패널은 루트 좌표계를 사용하고, 실로봇 지도
운용에서는 표시 좌표계를 `map`으로 바꾼다. ROS 환경에서는
`rviz2 -d <vision60_bringup-share>/config/operator.rviz`로 같은 3D 정보를
즉시 볼 수 있다.

## 셧다운 기간 오프라인 재생

통신복구 전 과정과 3D 현장 모델을 한 타임라인에서 확인할 때는 다음 통합
MCAP 하나만 연다.

```text
artifacts/integrated_calibrated_replay/integrated_operator_replay_0.mcap
```

LiDAR 907개, 카메라 454개와 로봇 위치를 통신복구 Mock에서 함께 기록하고,
같은 원본에서 컬러 점군·복셀·메쉬·실제 경로·통신 사건을 만들었다. 파생
장면의 원래 시각은 변경하지 않았다. Foxglove에서 3D 모델, 카메라,
실제·복귀 경로, 통신품질과 복구 상태를 동시에 재생할 수 있다. 원본 입력과
파생 결과의 해시 연결은 같은 폴더의 `lineage_validation.json`에서 확인한다.

Foxglove의 `Open local file(s)`에서 아래 두 MCAP을 동시에 선택하면 실제
Oxford 카메라·LiDAR·위치와 가공된 컬러 점군·경로를 하나의 시간축으로 본다.

```text
artifacts/oxford_spires_regression/converted/oxford_spires_sequence/
  oxford_spires_sequence_0.mcap
artifacts/oxford_spires_regression/command_view/command_view_replay/
  command_view_replay_0.mcap
```

통신 음영지역과 임무 사건 표시는 다음 Mock MCAP으로 확인한다.

```text
artifacts/scene_model_mock/command_view/command_view_replay/
  command_view_replay_0.mcap
```

통신두절부터 안전정지, 기록경로 복귀, 대체채널 전환, 동기화와 저속
재진입까지의 전체 상태 변화는 다음 MCAP으로 확인한다.

```text
artifacts/full_system_calibrated_mock_replay/
  full_system_calibrated_mock_replay_0.mcap
```

이 파일은 90.81초, 17개 토픽, 12,069개 메시지를 포함한다. `CameraInfo`와
`/tf_static`이 함께 저장되어 3D 파이프라인이 외부 보정 파일 없이 재생된다. 사용자 메시지
정의가 MCAP 내부에 저장되어 Foxglove가 별도 ROS 설치 없이 상태 필드를
읽을 수 있다. 생성 근거와 해시는 같은 폴더의 `replay_manifest.json`에 있다.
Foxglove에서 위 레이아웃을 선택하고 재생속도를 `10x`로 설정해 끝까지
재생하면 약 9초 안에 전체 상태 전이와 최종 `communication_transient_loss`
임무 사건을 확인할 수 있다.

## 자동 검증

```bash
./scripts/test_operator_visualization.sh
```

이 시험은 설정 구조, 읽기 전용 제한, ARM64 브리지 실행, 포트 8765의 실제
WebSocket 연결까지 확인한다. 전체 통신두절·역주행·대체채널 체인은 기존
`./scripts/test_full_system_mock.sh`가 검증한다.

기록 파일을 새로 만들고 재생 내용을 검증하는 명령은 다음과 같다.

```bash
./scripts/generate_full_system_mock_replay.sh
./scripts/test_full_system_mock_replay.sh
./scripts/generate_full_system_scene_artifacts.sh
./scripts/generate_integrated_operator_replay.sh
./scripts/test_integrated_operator_replay.sh
./scripts/generate_recovery_3d_command_view.sh
```
