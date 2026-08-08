# Mission Logger와 데이터 동기화

## 역할

통신두절 중 위치, 통신상태, 로봇 안전상태, 임무 사건과 복구 상태를
SQLite에 저장한다. 통신복구 후 `/mission/synchronize` Action 요청을
받으면 저장된 항목을 관제 수신기로 전송한다.

## 중복 방지

- 모든 항목에 안정적인 `item_id`와 SHA-256 체크섬을 부여한다.
- SQLite에서 `item_id`와 `(mission_id, checksum)`을 중복 제한한다.
- 수신 확인을 받은 항목만 `synced`로 변경한다.
- 실패 항목은 삭제하지 않고 다음 동기화에서 다시 시도한다.

## 현재 Mock 검증

```text
통신두절
→ SQLite 저장
→ 기록경로 역주행
→ 통신복구
→ SynchronizeMission Action
→ Mock 수신 확인
→ REENTRY_TEST
```

현재 전송기는 `MockSyncTransport`다. 실제 관제 API의 주소, 인증,
재시도 규약이 확정되면 transport만 교체한다. 원본 영상·점군은
SQLite에 넣지 않고 MCAP 파일로 기록한 뒤 파일 메타데이터와 체크섬을
동기화하는 방식으로 확장한다.
