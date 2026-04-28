#include <string.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#include "configblock.h"
#include "stabilizer_types.h"
#include "stabilizer.h"
#include "commander.h"

#include "app_config.h"
#include "ids.h"
#include "p2p_comm.h"
#include "cbba_full.h"

#define DEBUG_MODULE "APP"
#include "debug.h"

typedef enum
{
  ST_IDLE = 0,
  ST_TAKEOFF,
  ST_RUN,
  ST_LAND,
  ST_OFF
} AppState;

static CbbaState g_cbba;
static PeerSnapshotCache g_peer1;
static PeerSnapshotCache g_peer2;

static uint16_t u16_now_ms(TickType_t nowT)
{
  return (uint16_t)((nowT * portTICK_PERIOD_MS) & 0xFFFFu);
}

static uint8_t boolToU8(bool value)
{
  return value ? 1u : 0u;
}

static uint8_t execIsActive(uint8_t exec_task)
{
  return (exec_task < TASK_COUNT_RUNTIME) ? 1u : 0u;
}

static bool peerAliveId(uint8_t peer_radio_id, TickType_t nowT)
{
  const uint32_t lastMs = p2pCommGetLastRxMs(peer_radio_id);
  const uint32_t nowMs = (uint32_t)(nowT * portTICK_PERIOD_MS);

  if (lastMs == 0u)
  {
    return false;
  }

  return ((nowMs - lastMs) <= PEER_TO_MS);
}

static CbbaVec2 startPosFor(uint8_t radio_low)
{
  CbbaVec2 p;

  if (radio_low == NODE_ID_D1)
  {
    p.x_m = D1_X0_M;
    p.y_m = D1_Y0_M;
  }
  else if (radio_low == NODE_ID_D2)
  {
    p.x_m = D2_X0_M;
    p.y_m = D2_Y0_M;
  }
  else
  {
    p.x_m = D3_X0_M;
    p.y_m = D3_Y0_M;
  }

  return p;
}

static void updatePeerCache(uint8_t my_radio_low,
                            const msg_snapshot_frag_t *snapf)
{
  if (snapf->src_id == my_radio_low)
  {
    return;
  }

  if (my_radio_low == NODE_ID_D1)
  {
    if (snapf->src_id == NODE_ID_D2)
    {
      Cbba_UpdatePeerCacheFromFrag(&g_peer1, snapf);
    }
    else if (snapf->src_id == NODE_ID_D3)
    {
      Cbba_UpdatePeerCacheFromFrag(&g_peer2, snapf);
    }
  }
  else if (my_radio_low == NODE_ID_D2)
  {
    if (snapf->src_id == NODE_ID_D1)
    {
      Cbba_UpdatePeerCacheFromFrag(&g_peer1, snapf);
    }
    else if (snapf->src_id == NODE_ID_D3)
    {
      Cbba_UpdatePeerCacheFromFrag(&g_peer2, snapf);
    }
  }
  else
  {
    if (snapf->src_id == NODE_ID_D1)
    {
      Cbba_UpdatePeerCacheFromFrag(&g_peer1, snapf);
    }
    else if (snapf->src_id == NODE_ID_D2)
    {
      Cbba_UpdatePeerCacheFromFrag(&g_peer2, snapf);
    }
  }
}

static uint8_t execForRadio(uint8_t radio_low,
                            uint8_t my_radio_low,
                            const CbbaState *self,
                            const PeerSnapshotCache *peer1,
                            const PeerSnapshotCache *peer2)
{
  if (radio_low == my_radio_low)
  {
    return self->exec_task;
  }

  if ((peer1->valid != 0u) && (peer1->src_id == radio_low))
  {
    return peer1->exec_task;
  }

  if ((peer2->valid != 0u) && (peer2->src_id == radio_low))
  {
    return peer2->exec_task;
  }

  return 255u;
}

static void printObserverOneLine(uint8_t my_radio_low,
                                 bool aliveD1,
                                 bool aliveD2,
                                 bool aliveD3,
                                 uint8_t global_done_count,
                                 const CbbaObserverMetrics *om,
                                 const CbbaVec2 *worldPos)
{
  const uint8_t execD1 = execForRadio(NODE_ID_D1, my_radio_low, &g_cbba, &g_peer1, &g_peer2);
  const uint8_t execD2 = execForRadio(NODE_ID_D2, my_radio_low, &g_cbba, &g_peer1, &g_peer2);
  const uint8_t execD3 = execForRadio(NODE_ID_D3, my_radio_low, &g_cbba, &g_peer1, &g_peer2);

  const uint8_t activeD1 = execIsActive(execD1);
  const uint8_t activeD2 = execIsActive(execD2);
  const uint8_t activeD3 = execIsActive(execD3);

  const uint8_t active_any = ((activeD1 != 0u) || (activeD2 != 0u) || (activeD3 != 0u)) ? 1u : 0u;
  const uint8_t idle_all = ((activeD1 == 0u) && (activeD2 == 0u) && (activeD3 == 0u)) ? 1u : 0u;
  const uint8_t stuck = ((idle_all != 0u) && (global_done_count < TASK_COUNT_RUNTIME)) ? 1u : 0u;

  DEBUG_PRINT("[OBS_ONE] me=%s alive=(D1:%u,D2:%u,D3:%u) exec=(D1:%u,D2:%u,D3:%u) active_any=%u idle_all=%u stuck=%u done_global=%u/%u all_known=%u wconv=%u fpconv=%u contested=%u world_self=(%.2f,%.2f)\n",
              appNodeName(my_radio_low),
              (unsigned)boolToU8(aliveD1),
              (unsigned)boolToU8(aliveD2),
              (unsigned)boolToU8(aliveD3),
              (unsigned)execD1,
              (unsigned)execD2,
              (unsigned)execD3,
              (unsigned)active_any,
              (unsigned)idle_all,
              (unsigned)stuck,
              (unsigned)global_done_count,
              (unsigned)TASK_COUNT_RUNTIME,
              (unsigned)om->all_known,
              (unsigned)om->winner_conv,
              (unsigned)om->fp_conv,
              (unsigned)om->contested_tasks,
              (double)worldPos->x_m,
              (double)worldPos->y_m);
}

void appMain(void)
{
  const TickType_t dt = M2T(1000 / LOOP_HZ);

  uint8_t my_radio_low = 0u;
  uint8_t my_agent_id = 0u;

  AppState st = ST_IDLE;
  TickType_t st_t0 = 0;
  TickType_t stable_t0 = 0;
  TickType_t peer_loss_t0 = 0;
  TickType_t run_t0 = 0;

  TickType_t lastBeaconTx = 0;
  TickType_t lastSummary = 0;
  TickType_t lastStateLog = 0;
  TickType_t lastGuideLog = 0;
  TickType_t lastTableDump = 0;

  uint8_t seq_beacon = 0u;

  CbbaVec2 startPos;
  CbbaVec2 worldPos;

  vTaskDelay(M2T(4000));

  my_radio_low = (uint8_t)(configblockGetRadioAddress() & 0xFFu);

  if (!appIsValidNodeId(my_radio_low))
  {
    DEBUG_PRINT("[APP][ERR] invalid radio low=0x%02X\n", (unsigned)my_radio_low);

    while (1)
    {
      vTaskDelay(M2T(1000));
    }
  }

  my_agent_id = appAgentIdFromRadioLow(my_radio_low);
  startPos = startPosFor(my_radio_low);

  p2pCommInit(my_radio_low);
  Cbba_Init(&g_cbba, my_agent_id, startPos);
  Cbba_InitPeerCache(&g_peer1);
  Cbba_InitPeerCache(&g_peer2);

  DEBUG_PRINT("[APP] START my_id=0x%02X (%s) agent=%u start=(%.2f,%.2f)\n",
              (unsigned)my_radio_low,
              appNodeName(my_radio_low),
              (unsigned)my_agent_id,
              (double)startPos.x_m,
              (double)startPos.y_m);

  while (st != ST_OFF)
  {
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

    if ((st == ST_RUN) || (st == ST_LAND) || (st == ST_OFF))
    {
      worldPos.x_m = me.position.x + startPos.x_m;
      worldPos.y_m = me.position.y + startPos.y_m;
    }
    else
    {
      worldPos.x_m = startPos.x_m;
      worldPos.y_m = startPos.y_m;
    }

    Cbba_SetPose(&g_cbba, worldPos);

    if (my_radio_low == NODE_ID_D1)
    {
      aliveD1 = true;
      other_two_alive = aliveD2 && aliveD3;
    }
    else if (my_radio_low == NODE_ID_D2)
    {
      aliveD2 = true;
      other_two_alive = aliveD1 && aliveD3;
    }
    else
    {
      aliveD3 = true;
      other_two_alive = aliveD1 && aliveD2;
    }

    global_done_count = Cbba_GetGlobalDoneCount(&g_cbba, &g_peer1, &g_peer2);

    switch (st)
    {
      case ST_IDLE:
        if (other_two_alive)
        {
          if (stable_t0 == 0)
          {
            stable_t0 = nowT;
          }

          if ((nowT - stable_t0) > M2T(START_HOLD_MS))
          {
            st = ST_TAKEOFF;
            st_t0 = nowT;
            DEBUG_PRINT("[APP] -> TAKEOFF\n");
          }
        }
        else
        {
          stable_t0 = 0;
        }
        break;

      case ST_TAKEOFF:
        if ((nowT - st_t0) > M2T(TAKEOFF_MS))
        {
          st = ST_RUN;
          run_t0 = nowT;
          DEBUG_PRINT("[APP] -> RUN\n");
        }
        break;

      case ST_RUN:
        if (!other_two_alive)
        {
          if (peer_loss_t0 == 0)
          {
            peer_loss_t0 = nowT;
          }

          if ((nowT - peer_loss_t0) > M2T(PEER_LOSS_STREAK_MS))
          {
            st = ST_LAND;
            DEBUG_PRINT("[APP] peer lost(streak) -> LAND\n");
          }
        }
        else
        {
          peer_loss_t0 = 0;
        }

        if (global_done_count >= TASK_COUNT_RUNTIME)
        {
          if (g_cbba.mission_done_since_ms == 0u)
          {
            g_cbba.mission_done_since_ms = nowMs;
          }
          else if ((nowMs - g_cbba.mission_done_since_ms) >= MISSION_DONE_HOLD_MS)
          {
            st = ST_LAND;
            DEBUG_PRINT("[APP] all tasks done(global=%u) -> LAND\n",
                        (unsigned)global_done_count);
          }
        }
        else
        {
          g_cbba.mission_done_since_ms = 0u;
        }
        break;

      case ST_LAND:
        if (me.position.z < 0.08f)
        {
          st = ST_OFF;
          DEBUG_PRINT("[APP] -> OFF\n");
        }
        break;

      default:
        break;
    }

    if ((nowT - lastBeaconTx) >= M2T(1000 / BEACON_TX_HZ))
    {
      msg_beacon_t tx;

      memset(&tx, 0, sizeof(tx));

      lastBeaconTx = nowT;

      tx.type = MSG_BEACON;
      tx.src_id = my_radio_low;
      tx.tx_id = my_radio_low;
      tx.seq = seq_beacon++;
      tx.ttl = TTL_MAX;
      tx.hop = 0u;
      tx.t_ms = u16_now_ms(nowT);

      p2pCommSendBeacon(&tx);
    }

    if (st == ST_RUN)
    {
      app_rx_event_t ev;
      msg_claim_t claimMsg;
      msg_done_t doneMsg;
      msg_snapshot_frag_t snapFragMsg;

      while (p2pCommPollEvent(&ev))
      {
        if (ev.type == MSG_CLAIM)
        {
          Cbba_HandleClaim(&g_cbba, &ev.u.claim);
        }
        else if (ev.type == MSG_DONE)
        {
          Cbba_HandleDone(&g_cbba, &ev.u.done);
        }
        else if (ev.type == MSG_SNAPSHOT_FR)
        {
          Cbba_HandleSnapshotFrag(&g_cbba, &ev.u.snapf);
          updatePeerCache(my_radio_low, &ev.u.snapf);
        }
      }

      Cbba_LocalStep(&g_cbba, nowMs);
      Cbba_MarkReachedDone(&g_cbba, nowMs);

      if (Cbba_MakeClaimMsg(&g_cbba, nowMs, &claimMsg))
      {
        p2pCommSendClaim(&claimMsg);
      }

      if (Cbba_MakeDoneMsg(&g_cbba, nowMs, &doneMsg))
      {
        p2pCommSendDone(&doneMsg);
      }

      if (Cbba_MakeSnapshotFragMsg(&g_cbba, nowMs, &snapFragMsg))
      {
        p2pCommSendSnapshotFrag(&snapFragMsg);
      }

      if ((nowT - lastStateLog) >= M2T(STATE_LOG_MS))
      {
        lastStateLog = nowT;

        DEBUG_PRINT("[APP] state me=%s exec=%u done_local=%u done_global=%u world=(%.2f,%.2f)\n",
                    appNodeName(my_radio_low),
                    (unsigned)g_cbba.exec_task,
                    (unsigned)g_cbba.done_count,
                    (unsigned)global_done_count,
                    (double)worldPos.x_m,
                    (double)worldPos.y_m);
      }

      if ((nowT - lastSummary) >= M2T(SUMMARY_LOG_MS))
      {
        CbbaObserverMetrics om;

        lastSummary = nowT;

        Cbba_GetObserverMetrics(&g_cbba, &g_peer1, &g_peer2, &om);

        printObserverOneLine(my_radio_low,
                             aliveD1,
                             aliveD2,
                             aliveD3,
                             global_done_count,
                             &om,
                             &worldPos);

        DEBUG_PRINT("[OBS_DETAIL] me=%s all_known=%u equal=%u contested=%u wconv=%u fpconv=%u fp=(%lu,%lu,%lu) done_local=%u done_global=%u/%u rx=%lu drop=%lu\n",
                    appNodeName(my_radio_low),
                    (unsigned)om.all_known,
                    (unsigned)om.equal_winner_tasks,
                    (unsigned)om.contested_tasks,
                    (unsigned)om.winner_conv,
                    (unsigned)om.fp_conv,
                    (unsigned long)om.fp_shadow[0],
                    (unsigned long)om.fp_shadow[1],
                    (unsigned long)om.fp_shadow[2],
                    (unsigned)g_cbba.done_count,
                    (unsigned)global_done_count,
                    (unsigned)TASK_COUNT_RUNTIME,
                    (unsigned long)p2pCommGetRxCount(),
                    (unsigned long)p2pCommGetDropCount());
      }

      if ((TASK_TABLE_DEBUG_ENABLE != 0u) &&
          ((nowT - lastTableDump) >= M2T(TASK_TABLE_DEBUG_PERIOD_MS)))
      {
        lastTableDump = nowT;

        Cbba_DebugPrintTables(appNodeName(my_radio_low),
                              &g_cbba,
                              &g_peer1,
                              &g_peer2);
      }
    }

    if (st == ST_LAND)
    {
      sp.mode.z = modeVelocity;
      sp.velocity.z = LAND_VZ_MPS;
    }
    else if ((st == ST_TAKEOFF) || (st == ST_RUN))
    {
      sp.mode.z = modeAbs;
      sp.position.z = TAKEOFF_Z_M;

      if (st == ST_TAKEOFF)
      {
        sp.mode.x = modeAbs;
        sp.mode.y = modeAbs;
        sp.position.x = 0.0f;
        sp.position.y = 0.0f;
      }
      else
      {
        if ((nowT - run_t0) < M2T(POST_TAKEOFF_HOLD_MS))
        {
          sp.mode.x = modeAbs;
          sp.mode.y = modeAbs;
          sp.position.x = 0.0f;
          sp.position.y = 0.0f;
        }
        else if ((g_cbba.exec_task < g_cbba.task_count) &&
                 (nowMs >= g_cbba.replan_hold_until_ms))
        {
          const float tgt_local_x = g_cbba.tasks[g_cbba.exec_task].pos.x_m - startPos.x_m;
          const float tgt_local_y = g_cbba.tasks[g_cbba.exec_task].pos.y_m - startPos.y_m;
          const float ex = tgt_local_x - me.position.x;
          const float ey = tgt_local_y - me.position.y;
          const float dist2_local = (ex * ex) + (ey * ey);

          float vx = XY_KP * ex;
          float vy = XY_KP * ey;

          if (vx > XY_VEL_MAX)
          {
            vx = XY_VEL_MAX;
          }

          if (vx < -XY_VEL_MAX)
          {
            vx = -XY_VEL_MAX;
          }

          if (vy > XY_VEL_MAX)
          {
            vy = XY_VEL_MAX;
          }

          if (vy < -XY_VEL_MAX)
          {
            vy = -XY_VEL_MAX;
          }

          if ((nowT - lastGuideLog) >= M2T(300))
          {
            lastGuideLog = nowT;

            DEBUG_PRINT("[GUIDE] me=%s exec=%u tgt=(%.2f,%.2f) pos=(%.2f,%.2f) err=(%.2f,%.2f) dist2=%.3f vel=(%.2f,%.2f)\n",
                        appNodeName(my_radio_low),
                        (unsigned)g_cbba.exec_task,
                        (double)tgt_local_x,
                        (double)tgt_local_y,
                        (double)me.position.x,
                        (double)me.position.y,
                        (double)ex,
                        (double)ey,
                        (double)dist2_local,
                        (double)vx,
                        (double)vy);
          }

          if (dist2_local <= (GOAL_HOLD_RADIUS_M * GOAL_HOLD_RADIUS_M))
          {
            sp.mode.x = modeAbs;
            sp.mode.y = modeAbs;
            sp.position.x = tgt_local_x;
            sp.position.y = tgt_local_y;
          }
          else
          {
            sp.mode.x = modeVelocity;
            sp.mode.y = modeVelocity;
            sp.velocity.x = vx;
            sp.velocity.y = vy;
          }
        }
        else
        {
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

  DEBUG_PRINT("[APP] finished me=%s done_local=%u done_global=%u rx=%lu drop=%lu\n",
              appNodeName(my_radio_low),
              (unsigned)g_cbba.done_count,
              (unsigned)Cbba_GetGlobalDoneCount(&g_cbba, &g_peer1, &g_peer2),
              (unsigned long)p2pCommGetRxCount(),
              (unsigned long)p2pCommGetDropCount());

  while (1)
  {
    vTaskDelay(M2T(1000));
  }
}
