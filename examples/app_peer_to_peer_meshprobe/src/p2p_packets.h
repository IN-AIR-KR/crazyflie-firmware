#ifndef P2P_PACKETS_H
#define P2P_PACKETS_H

#include <stdint.h>

#include "app_config.h"

#define MSG_BEACON 1u
#define MSG_CBBA_STATE 2u

/* ── 위치 비콘: 9 bytes ───────────────────────────────────────────────────────
 * 모든 드론이 BEACON_TX_HZ Hz 로 브로드캐스트. 거리 제한 없음.
 * GS가 CRTP port 0x09 ch0(수신) / ch1(송신)으로 PC에 전달 → mesh_viz 시각화. */
typedef struct __attribute__((packed)) {
  uint8_t type; /* MSG_BEACON */
  uint8_t src_id;
  uint8_t seq;
  int16_t x_cm; /* 세계 좌표 (cm) */
  int16_t y_cm;
  int16_t z_cm;
} msg_beacon_t;

/* ── CBBA 상태 broadcast: 32 bytes (TASK_MAX=8, AGENT_COUNT=3) ───────────────
 * 각 에이전트가 CBBA_TX_PERIOD_MS 마다 자신의 전체 CBBA 상태를 브로드캐스트.
 * USE_CBBA_RANGE_LIMIT=1 이면 수신 측에서 peer 위치 캐시로 거리 필터 적용. */
typedef struct __attribute__((packed)) {
  uint8_t type; /* MSG_CBBA_STATE */
  uint8_t src_id;
  uint8_t seq;                    /* 단조증가 송신 카운터 (s_i 업데이트에 사용) */
  uint8_t exec_task;              /* 현재 수행 중인 task (없으면 255) */
  uint8_t done_mask;              /* bit k=1 이면 task k 완료 (TASK_MAX ≤ 8) */
  int16_t bid[TASK_MAX];          /* y_i: 태스크별 winning bid */
  uint8_t winner[TASK_MAX];       /* z_i: winner 에이전트 ID (0 = 없음) */
  uint8_t agent_seq[AGENT_COUNT]; /* s_i: 각 에이전트로부터 마지막 수신 seq */
} msg_cbba_state_t;

/* 컴파일 타임 크기 검증 — P2P 최대 60 bytes 제한 */
_Static_assert(sizeof(msg_beacon_t) == 9u, "msg_beacon_t must be 9 bytes");
_Static_assert(sizeof(msg_cbba_state_t) <= 60u, "msg_cbba_state_t exceeds P2P limit");

#endif
