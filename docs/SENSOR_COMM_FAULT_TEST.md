# 센서·통신 복합 고장 Mock 시험

실물 없이 LiDAR 지연·소실과 통신두절이 동시에 발생했을 때 최종 속도가
반드시 0이 되는지 확인한다.

ROS Tooling 공식 `topic_tools` Humble 1.1.2를 사용한다. `delay`는
`/ouster/points_raw`를 0.8초 늦춰 원래 timestamp 그대로
`/ouster/points`에 전달하고, `drop`은 `X=1`, `Y=1`로 모든 점군을
제거한다. 별도 Mock은 2초 후 통신을 끊고, 검증기는 0.2 m/s 명령을
계속 발행하면서 `/cmd_vel_safe`가 0인지 검사한다.

```text
Mock LiDAR → /ouster/points_raw → topic_tools delay/drop
                                   ↓
                            /ouster/points
                                   ↓
0.2 m/s 명령 → safety_velocity_gate → /cmd_vel_safe

vision60_mock → 정상 → 저하 → 통신두절
```

실행:

```bash
./scripts/test_sensor_comm_faults.sh
```

2026-07-31 결과:

- 0.8초 지연: 원본·지연 점군 수신, 최대 age 0.805초, 안전 출력 0, PASS
- 100% 손실: 원본 수신·출력 점군 없음, 안전 출력 0, PASS
- 관련 단위시험 45개: 실패 0, skip 2
- 전체 회귀시험 73개: 실패 0, skip 4

이 시험은 ROS 토픽 수준의 결정적 고장 주입이다. 학교 개방 후에는
실제 Ethernet 구간에 `tc netem`을 적용해 패킷 지연·손실도 별도로
검증한다.
