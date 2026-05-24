#ifndef CBBA_FULL_H
#define CBBA_FULL_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "p2p_packets.h"

typedef struct {
  float x_m;
  float y_m;
} CbbaVec2;

typedef struct {
  bool active;
  CbbaVec2 pos;
} CbbaTask;

typedef struct {
  uint8_t agent_id;
  uint8_t task_count;
  uint8_t bundle_limit;

  CbbaVec2 self_pos;
  CbbaTask tasks[TASK_MAX];

  /* CBBA 알고리즘 상태 (y_i, z_i, b_i, p_i, s_i) */
  int16_t bid_q[TASK_MAX];  /* y_i: winning bid per task */
  uint8_t winner[TASK_MAX]; /* z_i: winner agent ID (0 = nobody) */
  uint8_t bundle[TASK_MAX]; /* b_i: bundle (추가 순서) */
  uint8_t path[TASK_MAX];   /* p_i: path (방문 순서) */
  uint8_t bundle_len;
  uint8_t path_len;

  uint8_t last_seq[AGENT_COUNT]; /* s_i: peer별 마지막 수신 seq (Table 1 timestamp) */
  uint8_t my_seq;                /* 내 broadcast seq 카운터 */

  /* 완료 추적 */
  uint8_t done[TASK_MAX]; /* task별 완료 여부 */
  uint8_t done_mask;      /* 완료 task bitmask (1<<t, TASK_MAX<=8) */
  uint8_t done_count;
  uint32_t done_enter_ms[TASK_MAX];

  uint8_t exec_task; /* 현재 수행 task (없으면 255) */

  /* 타이밍 */
  uint32_t last_cbba_tx_ms;
  uint32_t replan_hold_until_ms;
  uint32_t mission_done_since_ms;
} CbbaState;

/* tasks 배열은 호출자(app_main.c)가 소유 — Init 시 내부로 복사됨 */
void Cbba_Init(CbbaState* s, uint8_t agent_id, CbbaVec2 start_pos, uint8_t task_count,
               uint8_t bundle_limit, const CbbaTask* tasks);
void Cbba_SetPose(CbbaState* s, CbbaVec2 pos);

/* Phase 1: bundle 갱신 (outbid 된 task suffix 해제 → addBundleTasks) */
void Cbba_LocalStep(CbbaState* s, uint32_t now_ms);

/* Phase 2: 이웃으로부터 수신한 상태로 Table 1 충돌 해결 + s_i 갱신 */
void Cbba_HandleCbbaState(CbbaState* s, const msg_cbba_state_t* m);

/* 현재 CBBA 상태를 패킷으로 직렬화 (CBBA_TX_PERIOD_MS 주기 체크 포함) */
bool Cbba_MakeCbbaStateMsg(CbbaState* s, uint32_t now_ms, msg_cbba_state_t* out);

/* exec_task에 충분히 가까이 접근했으면 done으로 표시 */
void Cbba_MarkReachedDone(CbbaState* s, uint32_t now_ms);

/* s->done[] 기반 전역 완료 카운트 (peer done_mask 반영 후 최신값) */
uint8_t Cbba_GetGlobalDoneCount(const CbbaState* s);

#endif
