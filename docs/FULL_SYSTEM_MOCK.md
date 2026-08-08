# Full-system Mock 체인

`vision60_bridge`가 유일한 로봇 경계가 되도록 Nav2 Mock 로봇을 제거하고,
안전 속도 명령부터 기록경로 역주행까지를 하나의 launch로 묶었다.

## 체인

```text
full_system_probe  (관제자 역할)
      │ /walk_enable=true      링크 정상 구간에서만 보행 승인
      │ /cmd_vel_autonomy      외행 주행 명령
      │ /cmd_vel_teleop        수동 우선권 시험
      ↓
twist_mux  (자율 50 < 수동 100 < 복귀 150)
      ↑ /cmd_vel_recovery      Nav2 기록경로 복귀·재진입
      ↑ /safety/cmd_vel_lock   안전 잠금 255
      ↓ /cmd_vel_muxed
nav2_velocity_smoother
      ↓ /cmd_vel_smoothed
Nav2 Collision Monitor
      ↓ /cmd_vel_collision_checked
safety_velocity_gate  (LiDAR 수신시각·header timestamp 확인)
      ↓ /cmd_vel_safe
vision60_bridge  (mock transport)
      ↓ /vision60/odom
robot_localization ekf_node
      ↓ /state/odometry + odom → base_link TF
      ├→ vision60_bridge      위치추정 heartbeat
      ├→ route_recorder       실제 통과경로 기록
      └→ nav2 controller_server / velocity_smoother

vision60_mock  → /communication/state (통신 시나리오만)
      ↓
comm_recovery_manager
      ↓ /vision60/request_safe_stop → 정지 확인 → RETURNING
      ↓ /vision60/set_walk_enabled(true)
route_recorder → /mission/recovery_path
      ↓
recovery_path_follower → Nav2 FollowPath → 역주행
      ↓
기존 채널 복구 실패 → CHANNEL_SWITCH
      ↓
mock_unavailable 실패 → mock_backup_wifi 성공
      ↓
채널 이상 후보 MissionEvent 기록

mission_logger → 통신두절 데이터 SQLite 저장
      ↓
통신복구 후 /mission/synchronize
      ↓
중복방지 동기화 완료 → REENTRY_TEST
      ↓
reentry_path_follower → 실제 복귀경로 정방향 재구성
      ↓
0.10 m/s 제한 FollowPath → 통신 유지 확인 → NORMAL
```

## 이전 Mock 구성과의 차이

| 항목 | 이전 | 현재 |
|---|---|---|
| 로봇 운동 | `nav2_mock_robot`이 `/cmd_vel_safe`를 직접 적분 | `vision60_bridge`의 mock transport가 적분 |
| 안전 게이트 | 우회됨 | 보행·비상정지·명령 timeout·센서 timestamp 전부 통과 |
| `/vision60/safety_state` | `vision60_mock`이 발행 | `vision60_bridge`가 단독 발행 |
| `/vision60/request_safe_stop` | `vision60_mock`이 제공 | `vision60_bridge`가 단독 제공 |
| 위치추정 | Mock 로봇이 TF 직접 발행 | `robot_localization`이 소유 |

`vision60_mock`은 `publish_robot_state:=false`일 때 통신 시나리오만
발행한다. 기본값은 `true`이므로 기존 시험은 그대로 동작한다.

## 보행 재활성화

통신두절 안전정지는 `walk_enabled`를 내린다. 이 상태로는 기록경로
역주행이 물리적으로 불가능하므로, `comm_recovery_manager`가
`STOPPING → RETURNING` 전이 **직후에만** `/vision60/set_walk_enabled(true)`를
호출한다. 정지 확인이 항상 먼저 오고, 재활성화는 복귀 주행에만 쓰인다.

`reenable_walk_for_recovery:=false`로 두면 재활성화를 하지 않고 관제자
승인을 기다린다.

## 실행

```bash
ros2 launch vision60_bringup full_system_mock.launch.py
```

기본값은 이동 출력을 차단한다. 자동시험에서만 다음처럼 허용한다.

```bash
ros2 launch vision60_bringup full_system_mock.launch.py \
  allow_motion_output:=true
```

회귀시험:

```bash
./scripts/test_full_system_mock.sh
```

관제용 MCAP 생성과 재생 검증:

```bash
./scripts/generate_full_system_mock_replay.sh
./scripts/test_full_system_mock_replay.sh
```

결과는 `artifacts/full_system_mock_replay`에 저장한다. 2026-07-31 생성본은
88.86초, 14개 토픽, 10,924개 메시지이며 사용자 메시지 정의와 SHA-256
manifest를 포함한다. 검증기는 파일을 다시 역직렬화하여
`LOST → STOPPING → RETURNING → CHANNEL_SWITCH → SYNCING → REENTRY_TEST →
NORMAL` 흐름과 첫 채널 실패·백업 채널 성공을 확인한다.

`full_system_probe`가 다음 항목을 모두 만족해야 PASS다.

1. `/vision60/odom`이 0.5 m 이상 전진 (bridge 게이트를 통과한 실제 이동)
2. 자율주행 명령과 수동 명령이 동시에 들어오면 수동 명령이 선택됨
3. `motion_allowed=false`에서 우선순위 255 안전 잠금이 발행됨
4. 감속 구역의 Mock 점군에서 최종 속도가 0.08 m/s 이하로 감소
5. 정지 구역의 Mock 점군에서 최종 속도가 0 m/s
6. 보행 승인 이후 `walk_enabled=false`이고 `motion_allowed=false`인
   정지 상태 관측
7. `RecoveryStatus`가 `RETURNING` 도달
8. `RETURNING` 이후 최대 도달점 대비 0.5 m 이상 후진
9. 통신복구 후 저장 데이터 동기화와 0.12 m/s 이하 재진입을 완료하고
   `NORMAL` 도달
10. 첫 대체채널 실패, 두 번째 대체채널 성공과 채널 이상 후보 기록

2026-07-31 최종 결과는 단위시험 73개(실패 0, skip 4), 수동 명령
우선 선택, 안전 잠금, 장애물 감속·정지, 외행 3.28 m, 안전 복귀,
첫 채널 실패, `mock_backup_wifi` 전환, 데이터 동기화 후 3.04 m까지
재진입했으며 재진입 최대속도 0.10 m/s로 PASS다.

## 명령 우선권 시험

공식 `twist_mux` 4.3.0을 사용해 자율주행 50, 수동조작 100, 복귀
150의 우선순위를 설정했다. 외행 중 0.20 m/s 자율 명령과 0.05 m/s
수동 명령을 동시에 보내 `/cmd_vel_muxed`가 수동 명령을 선택하는지
검사한다. `motion_lock_adapter`는 Vision60의 `motion_allowed`를
표준 Bool 잠금으로 변환한다. 잠금 우선순위는 255이고 0.5초 heartbeat
timeout을 사용하므로 어댑터가 멈춰도 잠금 상태가 된다.

## Mock 장애물 시험

`mock_lidar_heartbeat`가 실제 Ouster 대신 `/ouster/points`를 계속
발행한다. 빈 점군에서는 정상 주행하고, x=0.90 m에 점 5개를 넣으면
SlowdownZone이 30% 감속하며, x=0.40 m에 점 5개를 넣으면 StopZone이
속도를 0으로 만든다. 이후 다시 빈 점군으로 바꿔 주행이 계속되는지도
같은 전체 시험에서 확인한다. 내부 상태만 보지 않고 최종
`/cmd_vel_safe`를 검사하므로 안전 체인 전체를 검증한다.

## Mock 대체채널 시험

기본 Mock에서는 기존 채널을 끝까지 두절 상태로 유지한다.
`communication_channel_manager`가 `mock_unavailable`을 먼저 시도해
실패 결과를 받고, 다음 후보 `mock_backup_wifi`를 활성화한다. 전환
성공 후 `/communication/state`가 정상으로 바뀌어야 동기화와 재진입이
계속된다. 상태만 강제로 바꾸지 않고 실제 서비스 요청·응답과
`RecoveryEvent`를 모두 통과시킨다.

## 저속 재진입

`reentry_path_follower`는 저장된 복귀 경로를 다시 뒤집고 각 지점의
진행 방향을 재계산한다. Nav2 `SpeedLimit`으로 0.10 m/s를 적용하며,
재진입 중 통신두절이 설정시간 이상 반복되면 goal을 취소하고
`REENTRY_LINK_LOST`를 발행한다. 이 경우 상태기계는 해당 위치를 전파
음영 후보로 분류한다.

## 경로 시작 헤딩 정렬

복귀 경로는 항상 로봇 뒤에서 시작한다. DWB는 이런 경로를 제자리 회전으로
잡지 못하고 회전만 반복하다가 `SimpleProgressChecker`의 병진 미달 판정에
걸려 goal이 ABORTED(status=6) 된다. 실제로 10초 동안 회전 명령만 198회
발행하고 x가 전혀 변하지 않았다.

Nav2 공식 `RotationShimController`를 `FollowPath`의 상위 컨트롤러로 두어
경로 헤딩까지 먼저 제자리 회전한 뒤 DWB로 넘긴다.

```yaml
FollowPath:
  plugin: nav2_rotation_shim_controller::RotationShimController
  primary_controller: dwb_core::DWBLocalPlanner
  angular_dist_threshold: 0.785
  rotate_to_heading_angular_vel: 0.6   # bridge 각속도 한계와 동일
  max_angular_accel: 1.0
```

회전 구간에는 병진이 없으므로 `movement_time_allowance`를 10초에서
15초로 올렸다. 이 값을 다시 낮추면 회전이 끝나기 전에 중단된다.

## 시나리오 타이밍

`degraded_after_s`(기본 20)와 `disconnected_after_s`(기본 32)는 Nav2
lifecycle 활성화보다 길어야 한다. 그렇지 않으면 링크가 NORMAL인 동안
안전 경로점이 기록되지 않아 복귀 경로가 2점 미만으로 떨어진다.

`route_recorder`의 복귀 경로는 마지막 `safe_to_return` 지점까지만
되돌아간다. 따라서 외행 주행은 NORMAL 구간과 DEGRADED 구간에 모두
걸쳐 있어야 의미 있는 역주행 구간이 생긴다.

## 현재 체인에 없는 것

- `KrmVision60Interface`는 여전히 fail-closed placeholder다.
- `/vision60/odom` covariance는 SDK 확인 전까지 placeholder 파라미터
  (`odom_pose_variance`, `odom_twist_variance`)다. 전부 0이면 EKF가
  사용할 수 없어 기본값을 넣었을 뿐, 측정값이 아니다.
