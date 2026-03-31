#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app.h"
#include "configblock.h"
#include "crtp_commander_high_level.h"
#include "log.h"
#include "radiolink.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "task.h"

#define DEBUG_MODULE "CBAA"
#include "debug.h"

/* ═══════════════════════════════════════════════════════════
 * 상수
 * ═══════════════════════════════════════════════════════════ */
#define N_AGENTS 3
#define N_TASKS 3
#define CBAA_PORT 0x01      /* DTR=15, peer_to_peer=0 와 충돌 없음 */
#define CBAA_PERIOD_MS 200  /* CBAA 한 반복 주기 (ms) */
#define N_STABLE_REQUIRED 5 /* 수렴 판정용 연속 안정 반복 수 */
#define MAX_CBAA_ITER 100   /* 안전장치: 최대 반복 (100×200ms=20s) */

/* 비행 파라미터 */
#define TAKEOFF_HEIGHT 0.5f   /* 이륙 고도 (m) */
#define TAKEOFF_DURATION 2.0f /* 이륙 소요 시간 (s) */
#define APF_GOTO_DURATION 1.0f /* APF 목표 GoTo 지속 시간 (s); 200ms마다 갱신 */
#define LAND_DURATION 2.0f     /* 착륙 소요 시간 (s) */

/* APF 파라미터 */
#define APF_K_REP 0.05f          /* 척력 게인 */
#define APF_RHO_0 0.6f           /* 척력 영향 반경 (m) */
#define APF_MAX_REP 0.3f         /* 척력 최대 오프셋 크기 (m) */
#define APF_MIN_DIST 0.15f       /* 분모 0 방지용 최소 거리 (m) */
#define NEIGHBOR_TIMEOUT_MS 1000 /* 이 시간 내 수신 없으면 이웃 위치 무효 */

#define BID_INIT 0.0f           /* 초기 입찰가 (비음수) */
#define ARRIVAL_THRESHOLD 0.15f /* 도착 판정 거리 (m): 목표와의 3D 거리 */

/* ═══════════════════════════════════════════════════════════
 * 패킷 구조체 (CBAA 입찰 + 현재 위치 합산, 28 bytes)
 * ═══════════════════════════════════════════════════════════ */
typedef struct {
  uint8_t sender_id;                  /* [0]      라디오 주소 하위 바이트 */
  float y[N_TASKS];                   /* [1..12]  낙찰 최고가 (float×3, 12B) */
  uint8_t z_win[N_TASKS];             /* [13..15] 낙찰자 ID   (uint8×3,  3B) */
  float pos[3];                       /* [16..27] 현재 x,y,z  (float×3, 12B) */
} __attribute__((packed)) CbaaPacket; /* 총 28 bytes */

_Static_assert(sizeof(CbaaPacket) <= P2P_MAX_DATA_SIZE,
               "CbaaPacket exceeds P2P_MAX_DATA_SIZE");

/* ═══════════════════════════════════════════════════════════
 * 상태 머신
 * ═══════════════════════════════════════════════════════════ */
typedef enum {
  STATE_INIT = 0,
  STATE_WAIT_PEERS = 1, /* 모든 드론 온라인 확인 */
  STATE_TAKEOFF = 2,
  STATE_HOVER_WAIT = 3,
  STATE_CBAA_FLY = 4, /* CBAA + APF + 비행 동시 진행 */
  STATE_LAND_WAIT = 5,
  STATE_LAND = 6,
  STATE_DONE = 7,
} AppState;

/* ═══════════════════════════════════════════════════════════
 * 하드코딩 위치
 * ═══════════════════════════════════════════════════════════ */
/* 각 CF의 라디오 주소 하위 바이트 — 실제 기체 주소에 맞게 수정 */
static const uint8_t AGENT_IDS[N_AGENTS] = {0xE7, 0x08, 0x09};

/* 시작 위치: 지상 x-y (m), AGENT_IDS 순서와 동일 */
static const float START_POS[N_AGENTS][2] = {
    {0.0f, 0.0f}, /* ID 0xE7 → idx 0 */
    {0.5f, 0.0f}, /* ID 0x08 → idx 1 */
    {1.0f, 0.0f}, /* ID 0x09 → idx 2 */
};

/* 목표 대형: 절대 x, y, z (m) – 삼각형 */
static const float TASK_POS[N_TASKS][3] = {
    {0.0f, 0.5f, 0.5f}, /* Task 0 */
    {0.5f, 1.0f, 0.5f}, /* Task 1 */
    {1.0f, 0.5f, 0.5f}, /* Task 2 */
};

/* ═══════════════════════════════════════════════════════════
 * CBAA 상태 변수
 * ═══════════════════════════════════════════════════════════ */
static uint8_t my_id;
static uint8_t my_idx;

static uint8_t x_assign[N_TASKS]; /* 배정 벡터 (1 = 내가 담당) */
static float y_bid[N_TASKS];      /* 낙찰 최고가 (누적, 감소 없음) */
static uint8_t z_winner[N_TASKS]; /* 낙찰자 ID */
static float c[N_TASKS];          /* 점수 (매 반복 갱신) */
static int8_t assigned_task = -1;
static uint8_t cbaa_iter = 0;

/* 수렴 추적 */
static int8_t prev_assigned = -1;
static uint8_t stable_count = 0;

/* ═══════════════════════════════════════════════════════════
 * 이웃 위치 (APF용, 전역 좌표)
 * ═══════════════════════════════════════════════════════════ */
static float neighbor_pos[N_AGENTS][3];  /* [agent_idx][x,y,z] 전역 좌표 */
static uint32_t neighbor_tick[N_AGENTS]; /* 마지막 수신 tick */

/* 피어 감지 (WAIT_PEERS 단계) */
static volatile bool peer_heard[N_AGENTS];

/* ═══════════════════════════════════════════════════════════
 * 스레드 간 공유 (syslinkTask 콜백 ↔ appMain)
 * ═══════════════════════════════════════════════════════════ */
static volatile bool new_data_ready = false;
static CbaaPacket rx_shadow;

/* ═══════════════════════════════════════════════════════════
 * 앱 상태 + 로그 변수
 * ═══════════════════════════════════════════════════════════ */
static volatile AppState app_state = STATE_INIT;

static uint8_t log_state = 0;
static uint8_t log_task = 0xFF;
static uint8_t log_iter = 0;
static uint8_t log_stable = 0;
static float log_cx = 0.0f;
static float log_cy = 0.0f;
static float log_apf_tx = 0.0f;
static float log_apf_ty = 0.0f;
static float log_score0 = 0.0f;
static float log_score1 = 0.0f;
static float log_score2 = 0.0f;

/* ─────────────────────────────────────────────────────────── */
/* 내부 함수 선언                                              */
/* ─────────────────────────────────────────────────────────── */
static void cbaaReset(void);
static void cbaaUpdateScores(float cx, float cy);
static void cbaaAuction(void);
static void cbaaConsensus(const CbaaPacket* pkt);
static void cbaaBroadcast(float cx, float cy, float cz);
static bool cbaaPollRx(void);
static void apfCompute(float cx, float cy, float* out_tx, float* out_ty);
static void setAppState(AppState s);

/* ═══════════════════════════════════════════════════════════
 * P2P 콜백 (syslinkTask 컨텍스트)
 * ═══════════════════════════════════════════════════════════ */
static void p2pCallback(P2PPacket* p) {
  if (p->port != CBAA_PORT) return;
  if (p->size != sizeof(CbaaPacket)) return;

  CbaaPacket tmp;
  memcpy(&tmp, p->data, sizeof(CbaaPacket));

  if (tmp.sender_id == my_id) return; /* 자기 에코 무시 */

  /* WAIT_PEERS: 피어 감지만 수행 */
  if (app_state == STATE_WAIT_PEERS) {
    for (int k = 0; k < N_AGENTS; k++) {
      if (AGENT_IDS[k] == tmp.sender_id) {
        peer_heard[k] = true;
        break;
      }
    }
    return;
  }

  if (app_state != STATE_CBAA_FLY) return; /* 비행 중에만 CBAA 수용 */

  taskENTER_CRITICAL();
  memcpy(&rx_shadow, &tmp, sizeof(CbaaPacket));
  new_data_ready = true;
  taskEXIT_CRITICAL();
}

/* ═══════════════════════════════════════════════════════════
 * CBAA 초기화
 * ═══════════════════════════════════════════════════════════ */
static void cbaaReset(void) {
  for (int j = 0; j < N_TASKS; j++) {
    x_assign[j] = 0;
    y_bid[j] = BID_INIT;
    z_winner[j] = 0;
  }
  assigned_task = -1;
  prev_assigned = -1;
  stable_count = 0;
  cbaa_iter = 0;
  new_data_ready = false;
  log_task = 0xFF;
  log_iter = 0;
  log_stable = 0;
}

/* ═══════════════════════════════════════════════════════════
 * 점수 갱신: c_ij = 1 / (1 + dist_xy(curr, task_j))
 * ═══════════════════════════════════════════════════════════ */
static void cbaaUpdateScores(float cx, float cy) {
  for (int j = 0; j < N_TASKS; j++) {
    float dx = cx - TASK_POS[j][0];
    float dy = cy - TASK_POS[j][1];
    c[j] = 1.0f / (1.0f + sqrtf(dx * dx + dy * dy));
  }
  log_score0 = c[0];
  log_score1 = c[1];
  log_score2 = c[2];
}

/* ═══════════════════════════════════════════════════════════
 * Phase 1: 경매 — 미배정 시 최고 점수 태스크 입찰
 * ═══════════════════════════════════════════════════════════ */
static void cbaaAuction(void) {
  int sum = 0;
  for (int j = 0; j < N_TASKS; j++) sum += x_assign[j];
  if (sum > 0) return; /* 이미 배정받음 */

  int best_j = -1;
  float best = BID_INIT;
  for (int j = 0; j < N_TASKS; j++) {
    if (c[j] > y_bid[j] && c[j] > best) {
      best = c[j];
      best_j = j;
    }
  }
  if (best_j >= 0) {
    x_assign[best_j] = 1;
    y_bid[best_j] = c[best_j];
    z_winner[best_j] = my_id;
    assigned_task = best_j;
    log_task = (uint8_t)best_j;
    DEBUG_PRINT("[CBAA] bid task=%d score=%.4f\n", best_j, (double)c[best_j]);
  }
}

/* ═══════════════════════════════════════════════════════════
 * Phase 2: 합의 — 이웃 패킷으로 낙찰가·위치 갱신
 * ═══════════════════════════════════════════════════════════ */
static void cbaaConsensus(const CbaaPacket* pkt) {
  /* CBAA 입찰 합의 */
  for (int j = 0; j < N_TASKS; j++) {
    if (pkt->y[j] > y_bid[j]) {
      y_bid[j] = pkt->y[j];
      z_winner[j] = pkt->z_win[j];
      if (z_winner[j] != my_id) {
        x_assign[j] = 0;
        if (assigned_task == j) {
          assigned_task = -1;
          log_task = 0xFF;
          DEBUG_PRINT("[CBAA] lost task=%d to id=0x%02X\n", j, pkt->sender_id);
        }
      }
    }
  }

  /* 이웃 위치 갱신 (APF용): AGENT_IDS 테이블로 인덱스 조회 */
  for (int k = 0; k < N_AGENTS; k++) {
    if (AGENT_IDS[k] == pkt->sender_id) {
      neighbor_pos[k][0] = pkt->pos[0];
      neighbor_pos[k][1] = pkt->pos[1];
      neighbor_pos[k][2] = pkt->pos[2];
      neighbor_tick[k] = xTaskGetTickCount();
      break;
    }
  }
}

/* ═══════════════════════════════════════════════════════════
 * 브로드캐스트: 입찰 상태 + 현재 위치 전송
 * ═══════════════════════════════════════════════════════════ */
static void cbaaBroadcast(float cx, float cy, float cz) {
  static P2PPacket p2p_tx;
  static CbaaPacket pkt_tx;

  pkt_tx.sender_id = my_id;
  memcpy(pkt_tx.y, y_bid, sizeof(y_bid));
  memcpy(pkt_tx.z_win, z_winner, sizeof(z_winner));
  pkt_tx.pos[0] = cx;
  pkt_tx.pos[1] = cy;
  pkt_tx.pos[2] = cz;

  p2p_tx.port = CBAA_PORT;
  memcpy(p2p_tx.data, &pkt_tx, sizeof(CbaaPacket));
  p2p_tx.size = sizeof(CbaaPacket);

  radiolinkSendP2PPacketBroadcast(&p2p_tx);
}

/* ═══════════════════════════════════════════════════════════
 * 수신 폴링: 섀도우 버퍼에서 패킷 한 건 꺼내 처리
 * ═══════════════════════════════════════════════════════════ */
static bool cbaaPollRx(void) {
  taskENTER_CRITICAL();
  bool got = new_data_ready;
  CbaaPacket local;
  if (got) {
    memcpy(&local, &rx_shadow, sizeof(CbaaPacket));
    new_data_ready = false;
  }
  taskEXIT_CRITICAL();

  if (!got) return false;
  cbaaConsensus(&local);
  cbaaAuction();
  return true;
}

/* ═══════════════════════════════════════════════════════════
 * APF 보정 목표 계산 (xy 평면)
 * ═══════════════════════════════════════════════════════════ */
static void apfCompute(float cx, float cy, float* out_tx, float* out_ty) {
  if (assigned_task < 0) {
    *out_tx = cx;
    *out_ty = cy;
    return;
  }

  float goal_x = TASK_POS[assigned_task][0];
  float goal_y = TASK_POS[assigned_task][1];

  float rep_x = 0.0f;
  float rep_y = 0.0f;
  uint32_t now = xTaskGetTickCount();

  for (int k = 0; k < N_AGENTS; k++) {
    if (k == (int)my_idx) continue;

    if ((now - neighbor_tick[k]) > M2T(NEIGHBOR_TIMEOUT_MS)) continue;

    float dx = cx - neighbor_pos[k][0];
    float dy = cy - neighbor_pos[k][1];
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < APF_MIN_DIST) dist = APF_MIN_DIST;
    if (dist >= APF_RHO_0) continue;

    float factor = APF_K_REP * (1.0f / dist - 1.0f / APF_RHO_0) / (dist * dist);
    rep_x += factor * dx;
    rep_y += factor * dy;
  }

  float rep_mag = sqrtf(rep_x * rep_x + rep_y * rep_y);
  if (rep_mag > APF_MAX_REP) {
    float scale = APF_MAX_REP / rep_mag;
    rep_x *= scale;
    rep_y *= scale;
  }

  *out_tx = goal_x + rep_x;
  *out_ty = goal_y + rep_y;

  log_apf_tx = *out_tx;
  log_apf_ty = *out_ty;
}

/* ═══════════════════════════════════════════════════════════
 * 상태 전환 헬퍼
 * ═══════════════════════════════════════════════════════════ */
static void setAppState(AppState s) {
  app_state = s;
  log_state = (uint8_t)s;
  DEBUG_PRINT("[CBAA] state -> %d\n", (int)s);
}

/* ═══════════════════════════════════════════════════════════
 * appMain
 * ═══════════════════════════════════════════════════════════ */
void appMain(void) {
  /* ── INIT ─────────────────────────────────────────────── */
  uint64_t address = configblockGetRadioAddress();
  my_id = (uint8_t)(address & 0xFF);

  my_idx = N_AGENTS;
  for (int i = 0; i < N_AGENTS; i++) {
    if (AGENT_IDS[i] == my_id) {
      my_idx = (uint8_t)i;
      break;
    }
  }
  if (my_idx >= N_AGENTS) {
    DEBUG_PRINT("[CBAA] ERROR: my_id=0x%02X not in AGENT_IDS!\n", my_id);
    while (1) {
      vTaskDelay(M2T(1000));
    }
  }

  memset(neighbor_pos, 0, sizeof(neighbor_pos));
  memset(neighbor_tick, 0, sizeof(neighbor_tick));

  cbaaReset();
  p2pRegisterCB(p2pCallback);

  cbaaUpdateScores(START_POS[my_idx][0], START_POS[my_idx][1]);

  DEBUG_PRINT("[CBAA] Init: id=0x%02X idx=%d start=(%.2f,%.2f)\n", my_id,
              my_idx, (double)START_POS[my_idx][0],
              (double)START_POS[my_idx][1]);

  /* ── WAIT_PEERS ───────────────────────────────────────── */
  memset((void*)peer_heard, 0, sizeof(peer_heard));
  peer_heard[my_idx] = true;
  setAppState(STATE_WAIT_PEERS);
  DEBUG_PRINT("[CBAA] Waiting for all peers...\n");

  for (;;) {
    cbaaBroadcast(START_POS[my_idx][0], START_POS[my_idx][1], 0.0f);
    vTaskDelay(M2T(500));

    bool all_heard = true;
    for (int k = 0; k < N_AGENTS; k++) {
      if (!peer_heard[k]) {
        all_heard = false;
        break;
      }
    }
    if (all_heard) break;
  }
  DEBUG_PRINT("[CBAA] All peers online, proceeding to takeoff.\n");

  /* ── TAKEOFF ──────────────────────────────────────────── */
  setAppState(STATE_TAKEOFF);
  vTaskDelay(M2T(2000));

  crtpCommanderHighLevelTakeoff(TAKEOFF_HEIGHT, TAKEOFF_DURATION);
  while (!crtpCommanderHighLevelIsTrajectoryFinished()) {
    vTaskDelay(M2T(100));
  }

  /* ── HOVER_WAIT ───────────────────────────────────────── */
  setAppState(STATE_HOVER_WAIT);
  vTaskDelay(M2T(1000));

  /* ── CBAA_FLY ─────────────────────────────────────────── */
  setAppState(STATE_CBAA_FLY);
  cbaaReset();

  state_t cf_state;

  for (cbaa_iter = 0; cbaa_iter < MAX_CBAA_ITER; cbaa_iter++) {
    log_iter = cbaa_iter;

    /* 1. 현재 위치 취득 (stabilizerGetState 사용)
     *    stateEstimate.x/y = Optical flow 상대 좌표 (부팅 위치 = 0)
     *    전역 좌표 = 상대 좌표 + START_POS */
    stabilizerGetState(&cf_state);
    float cx_local = cf_state.position.x;
    float cy_local = cf_state.position.y;
    float cz       = cf_state.position.z;
    float cx = cx_local + START_POS[my_idx][0]; /* 전역 x */
    float cy = cy_local + START_POS[my_idx][1]; /* 전역 y */
    log_cx = cx;
    log_cy = cy;

    /* 2. 점수 동적 갱신 */
    cbaaUpdateScores(cx, cy);

    /* 3. Phase 1: 경매 */
    cbaaAuction();

    /* 4. 현재 입찰 + 위치 브로드캐스트 */
    cbaaBroadcast(cx, cy, cz);

    /* 5. CBAA_PERIOD_MS 동안 수신 패킷 처리 (10ms 폴링) */
    uint32_t period_end = xTaskGetTickCount() + M2T(CBAA_PERIOD_MS);
    while (xTaskGetTickCount() < period_end) {
      cbaaPollRx();
      vTaskDelay(M2T(10));
    }

    /* 6. APF 보정 목표 계산 */
    float apf_tx, apf_ty;
    apfCompute(cx, cy, &apf_tx, &apf_ty);

    float goal_z =
        (assigned_task >= 0) ? TASK_POS[assigned_task][2] : TAKEOFF_HEIGHT;

    /* 7. GoTo 재발행 (로컬 좌표로 변환) */
    if (assigned_task >= 0) {
      float goto_x = apf_tx - START_POS[my_idx][0];
      float goto_y = apf_ty - START_POS[my_idx][1];
      crtpCommanderHighLevelGoTo(goto_x, goto_y, goal_z, 0.0f,
                                 APF_GOTO_DURATION, false);
    }

    /* 8. 배정 안정성 추적 */
    if (assigned_task != prev_assigned) {
      if (assigned_task >= 0) {
        DEBUG_PRINT("[CBAA] iter=%d assign->task%d (%.2f,%.2f,%.2f)\n",
                    cbaa_iter, assigned_task,
                    (double)TASK_POS[assigned_task][0],
                    (double)TASK_POS[assigned_task][1],
                    (double)TASK_POS[assigned_task][2]);
      }
      prev_assigned = assigned_task;
      stable_count = 0;
    } else {
      if (stable_count < 255) stable_count++;
    }
    log_stable = stable_count;

    /* 9. 수렴 판정 */
    if (stable_count >= N_STABLE_REQUIRED && assigned_task >= 0) {
      float adx = cx - TASK_POS[assigned_task][0];
      float ady = cy - TASK_POS[assigned_task][1];
      float adz = cz - TASK_POS[assigned_task][2];
      float dist = sqrtf(adx * adx + ady * ady + adz * adz);
      if (dist < ARRIVAL_THRESHOLD) {
        DEBUG_PRINT("[CBAA] CONVERGED iter=%d task=%d dist=%.3f\n", cbaa_iter,
                    assigned_task, (double)dist);
        break;
      }
    }
  }

  /* 안전 폴백 */
  if (assigned_task < 0) {
    assigned_task = (int8_t)my_idx;
    log_task = (uint8_t)my_idx;
    DEBUG_PRINT("[CBAA] fallback task=%d\n", assigned_task);
  }

  /* 최종 GoTo */
  DEBUG_PRINT("[CBAA] FINAL task=%d pos=(%.2f,%.2f,%.2f)\n", assigned_task,
              (double)TASK_POS[assigned_task][0],
              (double)TASK_POS[assigned_task][1],
              (double)TASK_POS[assigned_task][2]);
  crtpCommanderHighLevelGoTo(
      TASK_POS[assigned_task][0] - START_POS[my_idx][0],
      TASK_POS[assigned_task][1] - START_POS[my_idx][1],
      TASK_POS[assigned_task][2], 0.0f, 5.0f, false);
  while (!crtpCommanderHighLevelIsTrajectoryFinished()) {
    vTaskDelay(M2T(100));
  }

  /* ── LAND_WAIT ────────────────────────────────────────── */
  setAppState(STATE_LAND_WAIT);
  vTaskDelay(M2T(3000));

  /* ── LAND ─────────────────────────────────────────────── */
  setAppState(STATE_LAND);
  crtpCommanderHighLevelLand(0.0f, LAND_DURATION);
  while (!crtpCommanderHighLevelIsTrajectoryFinished()) {
    vTaskDelay(M2T(100));
  }

  /* ── DONE ─────────────────────────────────────────────── */
  setAppState(STATE_DONE);
  DEBUG_PRINT("[CBAA] Done.\n");
  while (1) {
    vTaskDelay(M2T(1000));
  }
}

/* ═══════════════════════════════════════════════════════════
 * 로그 그룹 (cfclient 모니터링)
 * ═══════════════════════════════════════════════════════════ */
LOG_GROUP_START(cbaa)
LOG_ADD(LOG_UINT8, state, &log_state)
LOG_ADD(LOG_UINT8, task, &log_task)
LOG_ADD(LOG_UINT8, iter, &log_iter)
LOG_ADD(LOG_UINT8, stable, &log_stable)
LOG_ADD(LOG_FLOAT, posX, &log_cx)
LOG_ADD(LOG_FLOAT, posY, &log_cy)
LOG_ADD(LOG_FLOAT, apfTx, &log_apf_tx)
LOG_ADD(LOG_FLOAT, apfTy, &log_apf_ty)
LOG_ADD(LOG_FLOAT, score0, &log_score0)
LOG_ADD(LOG_FLOAT, score1, &log_score1)
LOG_ADD(LOG_FLOAT, score2, &log_score2)
LOG_GROUP_STOP(cbaa)
