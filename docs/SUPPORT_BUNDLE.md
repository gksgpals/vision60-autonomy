# Vision60 현장 지원 번들

실물 시험 중 문제가 생기면 원인을 재현할 수 있도록 로봇 HEALTH, 버전,
ROS 진단, rosbag metadata와 임무 DB를 한 ZIP으로 묶는다. 비밀번호·PSK·토큰·
private key로 보이는 텍스트 값은 자동으로 `[REDACTED]` 처리하며, 모든 파일은
SHA-256으로 기록한다.

```bash
python3 scripts/create_support_bundle.py \
  --output vision60_support_001.zip \
  --mission-id mission_001 \
  --robot-serial <SERIAL> \
  --input health=<HEALTH_EXPORT> \
  --input versions=config/first_day_field_capture.json \
  --input diagnostics=<ROS_DIAGNOSTICS_JSON> \
  --input mission_db=<MISSION_SQLITE3> \
  --input bag_metadata=<ROSBAG_METADATA_YAML>
```

ZIP은 전송 전에 사람이 한 번 열어 민감정보가 없는지 확인한다. 누락된 입력은
오류로 숨기지 않고 manifest에 `missing`으로 남긴다. Mock 번들 시험은
`scripts/test_support_bundle.py`로 재현할 수 있다.
