#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app.h"
#include "app_config.h"
#include "cbba_full.h"
#include "commander.h"
#include "configblock.h"
#include "ids.h"
#include "p2p_comm.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "task.h"

#define DEBUG_MODULE "APP"
#include "debug.h"

/* ── 상수 배열: task 좌표 & 드론 시작 위치 ───────────────────────────────
 * 개수를 늘리거나 줄이려면 이 배열과 TASK_COUNT_RUNTIME / AGENT_COUNT 만 변경. */

static const CbbaTask kTasks[TASK_COUNT_RUNTIME] = {
    {.active = true, .pos = {-0.90f, -0.85f}}, {.active = true, .pos = {-0.30f, 1.05f}},
    {.active = true, .pos = {0.95f, 0.80f}},   {.active = true, .pos = {1.10f, -0.35f}},
    {.active = true, .pos = {-1.15f, 0.25f}},  {.active = true, .pos = {0.20f, -1.10f}},
    {.active = true, .pos = {0.70f, 0.05f}},
};

/* 인덱스 0=D1, 1=D2, 2=D3 (appNodeIndexFromId 와 동일 순서) */
static const CbbaVec2 kStartPos[AGENT_COUNT] = {
    {0.00f, 0.00f}, /* D1 */
    {0.00f, 0.00f}, /* D2 */
    {0.00f, 0.00f}, /* D3 */
};

/* ── State Machine ──────────────────────────────────────────────────────── */
typedef enum { ST_IDLE = 0, ST_TAKEOFF, ST_RUN, ST_LAND, ST_OFF } AppState;

static CbbaState g_cbba;

static bool peerAliveId(uint8_t peer_radio_id, TickType_t nowT) {
  const uint32_t lastMs = p2pCommGetLastRxMs(peer_radio_id);
  const uint32_t nowMs = (uint32_t)(nowT * portTICK_PERIOD_MS);

  if (lastMs == 0u) {
    return false;
  }

  return ((nowMs - lastMs) <= PEER_TO_MS);
}

void appMain(void) {
  const TickType_t dt = M2T(1000 / LOOP_HZ);

  uint8_t my_radio_low = 0u;
  uint8_t my_agent_id = 0u;
  uint8_t my_start_idx = 0u;

  AppState st = ST_IDLE;
  TickType_t st_t0 = 0;
  TickType_t stable_t0 = 0;
  TickType_t peer_loss_t0 = 0;
  TickType_t run_t0 = 0;

  TickType_t lastBeaconTx = 0;
  TickType_t lastSummary = 0;
  TickType_t lastStateLog = 0;
  TickType_t lastTableDump = 0;

  uint8_t seq_beacon = 0u;
  uint32_t last_cbba_tx_ms = 0u;

  CbbaVec2 startPos;
  CbbaVec2 worldPos;

  vTaskDelay(M2T(4000));

  my_radio_low = (uint8_t)(configblockGetRadioAddress() & 0xFFu);

  if (!appIsValidNodeId(my_radio_low)) {
    DEBUG_PRINT("[APP][ERR] invalid radio low=0x%02X\n", (unsigned)my_radio_low);

    while (1) {
      vTaskDelay(M2T(1000));
    }
  }

  my_agent_id = appAgentIdFromRadioLow(my_radio_low);
  my_start_idx = appNodeIndexFromId(my_radio_low);
  startPos = kStartPos[my_start_idx];

  p2pCommInit(my_radio_low);
  Cbba_Init(&g_cbba, my_agent_id, startPos, TASK_COUNT_RUNTIME, BUNDLE_LIMIT, kTasks);

  DEBUG_PRINT("[APP] START my_id=0x%02X (%s) agent=%u start=(%.2f,%.2f)\n", (unsigned)my_radio_low,
              appNodeName(my_radio_low), (unsigned)my_agent_id, (double)startPos.x_m,
              (double)startPos.y_m);

  while (st != ST_OFF) {
    const TickType_t nowT = xTaskGetTickCount();
    const uint32_t nowMs = (uint32_t)(nowT * portTICK_PERIOD_MS);

    bool aliveD1 = peerAliveId(NODE_ID_D1, nowT);
    bool aliveD2 = peerAliveId(NODE_ID_D2, nowT);
    bool aliveD3 = peerAliveId(NODE_ID_D3, nowT);
    bool other_two_alive = false;

    state_t me;
    setpoint_t sp;
    uint8_t global_done_count = 0u;

    memset(&me, 0, sizeof(me));
    memset(&sp, 0, sizeof(sp));

    stabilizerGetState(&me);

    if ((st == ST_RUN) || (st == ST_LAND) || (st == ST_OFF)) {
      worldPos.x_m = me.position.x + startPos.x_m;
      worldPos.y_m = me.position.y + startPos.y_m;
    } else {
      worldPos.x_m = startPos.x_m;
      worldPos.y_m = startPos.y_m;
    }

    Cbba_SetPose(&g_cbba, worldPos);
    p2pCommSetLocalPos(me.position.x + startPos.x_m, me.position.y + startPos.y_m);

    if (my_radio_low == NODE_ID_D1) {
      aliveD1 = true;
      other_two_alive = aliveD2 && aliveD3;
    } else if (my_radio_low == NODE_ID_D2) {
      aliveD2 = true;
      other_two_alive = aliveD1 && aliveD3;
    } else {
      aliveD3 = true;
      other_two_alive = aliveD1 && aliveD2;
    }

    global_done_count = Cbba_GetGlobalDoneCount(&g_cbba);

    /* ── State Machine ─────────────────────────────────────────────────── */
    switch (st) {
      case ST_IDLE:
        if (other_two_alive) {
          if (stable_t0 == 0) {
            stable_t0 = nowT;
          }

#if MISSION_AUTO_START
          if ((nowT - stable_t0) > M2T(START_HOLD_MS)) {
            st = ST_TAKEOFF;
            st_t0 = nowT;
            DEBUG_PRINT("[APP] -> TAKEOFF\n");
          }
#endif
        } else {
          stable_t0 = 0;
        }
        break;

      case ST_TAKEOFF:
        if ((nowT - st_t0) > M2T(TAKEOFF_MS)) {
          st = ST_RUN;
          run_t0 = nowT;
          DEBUG_PRINT("[APP] -> RUN\n");
        }
        break;

      case ST_RUN:
        if (!other_two_alive) {
          if (peer_loss_t0 == 0) {
            peer_loss_t0 = nowT;
          }

          if ((nowT - peer_loss_t0) > M2T(PEER_LOSS_STREAK_MS)) {
            st = ST_LAND;
            DEBUG_PRINT("[APP] peer lost -> LAND\n");
          }
        } else {
          peer_loss_t0 = 0;
        }

        if (global_done_count >= TASK_COUNT_RUNTIME) {
          if (g_cbba.mission_done_since_ms == 0u) {
            g_cbba.mission_done_since_ms = nowMs;
          } else if ((nowMs - g_cbba.mission_done_since_ms) >= MISSION_DONE_HOLD_MS) {
            st = ST_LAND;
            DEBUG_PRINT("[APP] all tasks done(global=%u) -> LAND\n", (unsigned)global_done_count);
          }
        } else {
          g_cbba.mission_done_since_ms = 0u;
        }
        break;

      case ST_LAND:
        if (me.position.z < 0.08f) {
          st = ST_OFF;
          DEBUG_PRINT("[APP] -> OFF\n");
        }
        break;

      default:
        break;
    }

    /* ── 비콘 송신 ──────────────────────────────────────────────────────── */
    if ((nowT - lastBeaconTx) >= M2T(1000 / BEACON_TX_HZ)) {
      msg_beacon_t tx;

      memset(&tx, 0, sizeof(tx));

      lastBeaconTx = nowT;

      tx.type = MSG_BEACON;
      tx.src_id = my_radio_low;
      tx.seq = seq_beacon++;
      tx.x_cm = (int16_t)((me.position.x + startPos.x_m) * 100.0f);
      tx.y_cm = (int16_t)((me.position.y + startPos.y_m) * 100.0f);
      tx.z_cm = (int16_t)(me.position.z * 100.0f);

      p2pCommSendBeacon(&tx);
    }

    /* ── CBBA 루프 (ST_RUN 에서만) ──────────────────────────────────────── */
    if (st == ST_RUN) {
      msg_cbba_state_t cbba_msg;

      /* Phase 2: 수신된 상태 처리 */
      while (p2pCommPollCbbaState(&cbba_msg)) {
        Cbba_HandleCbbaState(&g_cbba, &cbba_msg);
      }

      /* Phase 1: bundle 재구성 */
      Cbba_LocalStep(&g_cbba, nowMs);
      Cbba_MarkReachedDone(&g_cbba, nowMs);

      /* CBBA 상태 broadcast */
      if ((nowMs - last_cbba_tx_ms) >= CBBA_TX_PERIOD_MS) {
        msg_cbba_state_t out;

        if (Cbba_MakeCbbaStateMsg(&g_cbba, nowMs, &out)) {
          p2pCommSendCbbaState(&out);
        }

        last_cbba_tx_ms = nowMs;
      }

      /* ── 진단 로그 ─────────────────────────────────────────────────── */
      if ((nowT - lastStateLog) >= M2T(STATE_LOG_MS)) {
        lastStateLog = nowT;

        DEBUG_PRINT(
            "[APP] me=%s exec=%u done_local=%u done_global=%u "
            "world=(%.2f,%.2f)\n",
            appNodeName(my_radio_low), (unsigned)g_cbba.exec_task, (unsigned)g_cbba.done_count,
            (unsigned)global_done_count, (double)worldPos.x_m, (double)worldPos.y_m);
      }

      if ((nowT - lastSummary) >= M2T(SUMMARY_LOG_MS)) {
        lastSummary = nowT;

        DEBUG_PRINT(
            "[OBS] me=%s alive=(D1:%u,D2:%u,D3:%u) exec=%u "
            "done_global=%u/%u rx=%lu\n",
            appNodeName(my_radio_low), (unsigned)aliveD1, (unsigned)aliveD2, (unsigned)aliveD3,
            (unsigned)g_cbba.exec_task, (unsigned)global_done_count, (unsigned)TASK_COUNT_RUNTIME,
            (unsigned long)p2pCommGetRxCount());
      }

      if ((TASK_TABLE_DEBUG_ENABLE != 0u) &&
          ((nowT - lastTableDump) >= M2T(TASK_TABLE_DEBUG_PERIOD_MS))) {
        uint8_t t = 0u;

        lastTableDump = nowT;

        DEBUG_PRINT(
            "[CBBA_DUMP] me=%s agent=%u exec=%u done=%u/%u "
            "bundle_len=%u path_len=%u\n",
            appNodeName(my_radio_low), (unsigned)g_cbba.agent_id, (unsigned)g_cbba.exec_task,
            (unsigned)g_cbba.done_count, (unsigned)g_cbba.task_count, (unsigned)g_cbba.bundle_len,
            (unsigned)g_cbba.path_len);

        for (t = 0u; t < g_cbba.task_count; t++) {
          DEBUG_PRINT(
              "[CBBA_TASK] t=%u done=%u winner=%u bid=%d "
              "pos=(%.2f,%.2f)\n",
              (unsigned)t, (unsigned)g_cbba.done[t], (unsigned)g_cbba.winner[t],
              (int)g_cbba.bid_q[t], (double)g_cbba.tasks[t].pos.x_m,
              (double)g_cbba.tasks[t].pos.y_m);
        }
      }
    }

    /* ── 비행 setpoint ───────────────────────────────────────────────────── */
    if (st == ST_LAND) {
      sp.mode.z = modeVelocity;
      sp.velocity.z = LAND_VZ_MPS;
    } else if ((st == ST_TAKEOFF) || (st == ST_RUN)) {
      sp.mode.z = modeAbs;
      sp.position.z = TAKEOFF_Z_M;

      if (st == ST_TAKEOFF) {
        sp.mode.x = modeAbs;
        sp.mode.y = modeAbs;
        sp.position.x = 0.0f;
        sp.position.y = 0.0f;
      } else {
        if ((nowT - run_t0) < M2T(POST_TAKEOFF_HOLD_MS)) {
          sp.mode.x = modeAbs;
          sp.mode.y = modeAbs;
          sp.position.x = 0.0f;
          sp.position.y = 0.0f;
        } else if ((g_cbba.exec_task < g_cbba.task_count) &&
                   (nowMs >= g_cbba.replan_hold_until_ms)) {
          const float tgt_local_x = g_cbba.tasks[g_cbba.exec_task].pos.x_m - startPos.x_m;
          const float tgt_local_y = g_cbba.tasks[g_cbba.exec_task].pos.y_m - startPos.y_m;
          const float ex = tgt_local_x - me.position.x;
          const float ey = tgt_local_y - me.position.y;
          const float dist2_local = (ex * ex) + (ey * ey);

          float vx = XY_KP * ex;
          float vy = XY_KP * ey;

          if (vx > XY_VEL_MAX)
            vx = XY_VEL_MAX;
          if (vx < -XY_VEL_MAX)
            vx = -XY_VEL_MAX;
          if (vy > XY_VEL_MAX)
            vy = XY_VEL_MAX;
          if (vy < -XY_VEL_MAX)
            vy = -XY_VEL_MAX;

          if (dist2_local <= (GOAL_HOLD_RADIUS_M * GOAL_HOLD_RADIUS_M)) {
            sp.mode.x = modeAbs;
            sp.mode.y = modeAbs;
            sp.position.x = tgt_local_x;
            sp.position.y = tgt_local_y;
          } else {
            sp.mode.x = modeVelocity;
            sp.mode.y = modeVelocity;
            sp.velocity.x = vx;
            sp.velocity.y = vy;
          }
        } else {
          sp.mode.x = modeAbs;
          sp.mode.y = modeAbs;
          sp.position.x = me.position.x;
          sp.position.y = me.position.y;
        }
      }
    }

    commanderSetSetpoint(&sp, 3);
    vTaskDelay(dt);
  }

  DEBUG_PRINT("[APP] finished me=%s done_count=%u rx=%lu\n", appNodeName(my_radio_low),
              (unsigned)g_cbba.done_count, (unsigned long)p2pCommGetRxCount());

  while (1) {
    vTaskDelay(M2T(1000));
  }
}
