# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 코드 구조 — meshprobe 앱

이 레포는 Crazyflie 공식 펌웨어를 베이스로 하고, `examples/app_peer_to_peer_meshprobe/` 가 out-of-tree app으로 올라타는 구조다.

```
examples/app_peer_to_peer_meshprobe/src/
  app_config.h   — 실험 파라미터 전부 (좌표, 타이밍, 플래그)
  app_main.c     — 앱 진입점, State Machine, 비행 세트포인트
  p2p_comm.c/h   — P2P 수신 콜백, 중복 제거(seen), 이벤트 큐, 송신 래퍼
  p2p_packets.h  — 모든 P2P 메시지 구조체
  cbba_full.c/h  — 분산 task 할당 (CBBA), 경로 계산, done 판정
  cbba_state.c/h — observer 메트릭, peer cache
  ids.h          — D1/D2/D3 노드 ID ↔ 이름 변환 (inline)

test_python/mesh_viz/
  app.py                — Flask-SocketIO 시각화 서버
  templates/index.html  — 실시간 그래프 UI (WebSocket)
```

### 통신 스택

```
드론 ←→ 드론 : P2P (nRF ESB broadcast, radiolinkSendP2PPacketBroadcast)
드론 → PC    : CRTP port 0x09
  channel 0 = 다른 드론에게서 받은 beacon을 PC로 중계
  channel 1 = GS 드론 자신이 송신한 beacon을 PC에 알림
```

`p2pRxCb` 처리 순서:

1. 자기 패킷 / 유효하지 않은 노드 ID → 무시
2. seen 캐시로 중복 제거
3. `USE_RANGE_LIMIT=1`이면 beacon의 x_cm/y_cm로 거리 계산, COMM_RADIUS_M 초과 시 드랍
4. CLAIM/DONE/SNAPSHOT_FR → 이벤트 큐 push
5. MSG_BEACON → CRTP channel 0으로 PC 전달

### 위치 데이터 흐름

`me.position` (Crazyflie 내부 좌표) + `startPos` (세계 좌표 오프셋) = 세계 좌표 `worldPos`

비콘 패킷 `x_cm = (me.position.x + startPos.x_m) * 100` — `worldPos`가 아닌 직접 계산. `worldPos`는 ST_RUN/LAND/OFF에서만 갱신되므로 idle 중 비콘에 반영하기 위한 의도적 선택.

### 주요 설정 플래그 (`app_config.h`)

| 플래그               | 기본값  | 설명                                              |
| -------------------- | ------- | ------------------------------------------------- |
| `MISSION_AUTO_START` | `0u`    | 0=mesh 통신만 테스트(이륙 없음), 1=자동 이륙+미션 |
| `USE_RANGE_LIMIT`    | `0u`    | 1이면 COMM_RADIUS_M 밖 beacon 드랍                |
| `COMM_RADIUS_M`      | `1.00f` | 통신 반경 (m)                                     |
| `TTL_MAX`            | `1u`    | P2P 홉 제한                                       |
| `TASK_COUNT_RUNTIME` | `7u`    | 실행할 task 수                                    |

### State Machine (app_main.c)

`ST_IDLE → ST_TAKEOFF → ST_RUN → ST_LAND → ST_OFF`

- `MISSION_AUTO_START=0`이면 IDLE에서 TAKEOFF로 전이하지 않음 (mesh 통신만 동작)
- TAKEOFF 후 `POST_TAKEOFF_HOLD_MS` 동안은 제자리 유지, 이후 task 추적 시작
- `global_done_count >= TASK_COUNT_RUNTIME`이 되면 `MISSION_DONE_HOLD_MS` 대기 후 LAND
