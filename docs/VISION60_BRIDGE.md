# Vision60 Bridge

## 역할

`vision60_bridge`는 ROS 2 안전 속도 명령과 KRM SDK 사이의 경계다.
현재는 SDK 대신 `MockVision60Interface`를 사용한다.

```text
/cmd_vel_safe
      ↓
명령 검증·timeout·상태 gate
      ↓
Vision60Interface
├─ MockVision60Interface  # 현재 자동시험
└─ KrmVision60Interface   # SDK 수신 후 구현
```

## 이동 허용 조건

다음 조건이 모두 참일 때만 0이 아닌 속도를 transport에 전달한다.

- `allow_motion_output=true`
- 보행 활성
- 비상정지 미발생
- transport 연결 정상
- `/state/odometry` timestamp 정상
- `/ouster/points` timestamp 정상
- `/cmd_vel_safe`가 유효하고 timeout 이내

NaN, Inf 또는 제한을 넘는 명령은 전체 명령을 거부한다. 이동 중 조건
하나라도 깨지면 가속도 완화를 기다리지 않고 즉시 0 속도를 보낸다.

## ROS 인터페이스

입력:

- `/cmd_vel_safe` (`geometry_msgs/Twist`)
- `/walk_enable` (`std_msgs/Bool`)
- `/emergency_stop` (`std_msgs/Bool`, true는 latch)
- `/state/odometry` (`nav_msgs/Odometry`)
- `/ouster/points` (`sensor_msgs/PointCloud2`)

서비스:

- `/vision60/set_walk_enabled`
- `/vision60/request_safe_stop`

출력:

- `/vision60/odom`
- `/vision60/safety_state`
- `/vision60/state`
- `/vision60/battery`
- `/vision60/fault`
- `/vision60/estop_state`
- `/vision60/command_applied`

## 안전한 실행

기본 launch는 이동 출력을 차단한다.

```bash
ros2 launch vision60_bringup vision60_bridge_mock.launch.py
```

Mock 자동시험에서만 다음처럼 허용한다.

```bash
ros2 launch vision60_bringup vision60_bridge_mock.launch.py \
  allow_motion_output:=true
```

전체 자동시험:

```bash
./scripts/test_vision60_bridge.sh
```

실물에서 `allow_motion_output=true`를 사용하기 전에 KRM SDK 단독 정지
시험, 물리 E-stop, 속도 제한, 장착 TF와 센서 timestamp를 확인한다.
