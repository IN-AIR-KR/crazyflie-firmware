#include <math.h>
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

typedef enum { ST_IDLE = 0, ST_TAKEOFF, ST_RUN, ST_LAND, ST_OFF } AppState;

static CbbaState g_cbba;
static PeerMissionCache g_peer1;
static PeerMissionCache g_peer2;

static uint16_t u16_now_ms(TickType_t nowT) {
  return (uint16_t)((nowT * portTICK_PERIOD_MS) & 0xFFFFu);
}

static uint8_t timeUntilActive(uint32_t now_ms, uint32_t until_ms) {
  if (until_ms == 0u) {
    return 0u;
  }

  return (((int32_t)(until_ms - now_ms)) > 0) ? 1u : 0u;
}

static uint16_t cmMetricFromMeter(float v_m) {
  const float cm_f = v_m * 100.0f;

  if (cm_f <= 0.0f) {
    return 0u;
  }

  if (cm_f >= 65535.0f) {
    return 65535u;
  }

  return (uint16_t)lrintf(cm_f);
}

static uint8_t u8Saturate(uint16_t v) {
  return (v > 255u) ? 255u : (uint8_t)v;
}

static bool peerAliveId(uint8_t peer_radio_id, TickType_t nowT) {
  const uint32_t lastMs = p2pCommGetLastRxMs(peer_radio_id);
  const uint32_t nowMs = (uint32_t)(nowT * portTICK_PERIOD_MS);

  if (lastMs == 0u) {
    return false;
  }

  return ((nowMs - lastMs) <= PEER_TO_MS);
}

static bool peerStartReadyId(uint8_t peer_radio_id) {
  uint8_t app_state = 0u;
  uint8_t start_ready = 0u;
  uint32_t age_ms = 0u;

  if (!p2pCommGetPeerStartInfo(peer_radio_id, &app_state, &start_ready, &age_ms)) {
    return false;
  }

  return ((start_ready != 0u) && (age_ms <= PEER_TO_MS) &&
          ((app_state == (uint8_t)ST_IDLE) || (app_state == (uint8_t)ST_TAKEOFF)));
}

static bool peerTakeoffSyncId(uint8_t peer_radio_id) {
  uint8_t app_state = 0u;
  uint8_t start_ready = 0u;
  uint32_t age_ms = 0u;

  if (!p2pCommGetPeerStartInfo(peer_radio_id, &app_state, &start_ready, &age_ms)) {
    return false;
  }

  (void)start_ready;

  return ((age_ms <= PEER_TO_MS) &&
          ((app_state == (uint8_t)ST_TAKEOFF) || (app_state == (uint8_t)ST_RUN)));
}

static CbbaVec2 startPosFor(uint8_t radio_low) {
  CbbaVec2 p;

  if (radio_low == NODE_ID_D1) {
    p.x_m = D1_X0_M;
    p.y_m = D1_Y0_M;
  } else if (radio_low == NODE_ID_D2) {
    p.x_m = D2_X0_M;
    p.y_m = D2_Y0_M;
  } else {
    p.x_m = D3_X0_M;
    p.y_m = D3_Y0_M;
  }

  return p;
}

static void clampVelocity2(float* vx, float* vy, float max_v) {
  const float v2 = ((*vx) * (*vx)) + ((*vy) * (*vy));

  if (v2 > (max_v * max_v)) {
    const float scale = max_v / sqrtf(v2);
    *vx *= scale;
    *vy *= scale;
  }
}

static void addPeerAvoidance(uint8_t my_radio_low, const CbbaVec2* worldPos, float* vx, float* vy) {
#if APF_AVOID_ENABLE
  const uint8_t peer_ids[AGENT_COUNT] = {NODE_ID_D1, NODE_ID_D2, NODE_ID_D3};
  float ax = 0.0f;
  float ay = 0.0f;
  uint8_t hard_avoid = 0u;
  uint8_t i = 0u;

  for (i = 0u; i < AGENT_COUNT; i++) {
    float px = 0.0f;
    float py = 0.0f;
    uint32_t age_ms = 0u;
    float dx = 0.0f;
    float dy = 0.0f;
    float d2 = 0.0f;
    float d = 0.0f;
    float mag = 0.0f;

    if (peer_ids[i] == my_radio_low) {
      continue;
    }

    if (!p2pCommGetPeerPos(peer_ids[i], &px, &py, &age_ms)) {
      continue;
    }

    if (age_ms > APF_PEER_POS_TIMEOUT_MS) {
      continue;
    }

    dx = worldPos->x_m - px;
    dy = worldPos->y_m - py;
    d2 = (dx * dx) + (dy * dy);

    if (d2 >= (APF_RADIUS_M * APF_RADIUS_M)) {
      continue;
    }

    if (d2 < 0.0001f) {
      dx = (my_radio_low < peer_ids[i]) ? -1.0f : 1.0f;
      dy = 0.0f;
      d = 0.01f;
    } else {
      d = sqrtf(d2);
      dx /= d;
      dy /= d;
    }

    if (d < APF_HARD_RADIUS_M) {
      hard_avoid = 1u;
      mag = APF_VEL_MAX;
    } else {
      const float soft = (APF_RADIUS_M - d) / (APF_RADIUS_M - APF_HARD_RADIUS_M);
      mag = APF_VEL_MAX * soft * soft;
    }

    ax += dx * mag;
    ay += dy * mag;
  }

  clampVelocity2(&ax, &ay, APF_VEL_MAX);

  if (hard_avoid != 0u) {
    *vx = ax;
    *vy = ay;
  } else {
    *vx += ax;
    *vy += ay;
  }

  clampVelocity2(vx, vy, XY_VEL_MAX);
#else
  (void)my_radio_low;
  (void)worldPos;
  (void)vx;
  (void)vy;
#endif
}

static void applyIdleAvoidanceSetpoint(setpoint_t* sp, uint8_t my_radio_low,
                                       const CbbaVec2* worldPos) {
  float vx = 0.0f;
  float vy = 0.0f;

  addPeerAvoidance(my_radio_low, worldPos, &vx, &vy);

  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->velocity.x = vx;
  sp->velocity.y = vy;
}
static PeerMissionCache* peerCacheForSource(uint8_t my_radio_low, uint8_t src_id) {
  if (src_id == my_radio_low) {
    return (PeerMissionCache*)0;
  }

  if (my_radio_low == NODE_ID_D1) {
    if (src_id == NODE_ID_D2) {
      return &g_peer1;
    }

    if (src_id == NODE_ID_D3) {
      return &g_peer2;
    }
  } else if (my_radio_low == NODE_ID_D2) {
    if (src_id == NODE_ID_D1) {
      return &g_peer1;
    }

    if (src_id == NODE_ID_D3) {
      return &g_peer2;
    }
  } else {
    if (src_id == NODE_ID_D1) {
      return &g_peer1;
    }

    if (src_id == NODE_ID_D2) {
      return &g_peer2;
    }
  }

  return (PeerMissionCache*)0;
}

static void updatePeerCacheFromClaimLite(uint8_t my_radio_low, const msg_claim_t* claim) {
  PeerMissionCache* c = peerCacheForSource(my_radio_low, claim->src_id);

  if (c != (PeerMissionCache*)0) {
    Cbba_UpdatePeerCacheFromClaim(c, claim);
  }
}

static void updatePeerCacheFromDoneLite(uint8_t my_radio_low, const msg_done_t* done) {
  PeerMissionCache* c = peerCacheForSource(my_radio_low, done->src_id);

  if (c != (PeerMissionCache*)0) {
    Cbba_UpdatePeerCacheFromDone(c, done);
  }
}

static void updatePeerCacheFromBidVecLite(uint8_t my_radio_low, const msg_bidvec_t* bidv) {
  PeerMissionCache* c = peerCacheForSource(my_radio_low, bidv->src_id);

  if (c != (PeerMissionCache*)0) {
    Cbba_UpdatePeerCacheFromBidVec(c, bidv);
  }
}

void appMain(void) {
  const TickType_t dt = M2T(1000 / LOOP_HZ);

  uint8_t my_radio_low = 0u;
  uint8_t my_agent_id = 0u;

  AppState st = ST_IDLE;
  TickType_t st_t0 = 0;
  TickType_t run_t0 = 0;
  TickType_t stable_t0 = 0;
  TickType_t peer_loss_t0 = 0;

  TickType_t lastBeaconTx = 0;

  uint8_t seq_beacon = 0u;
  uint8_t distMetricValid = 0u;
  float totalDistanceM = 0.0f;

  CbbaVec2 startPos;
  CbbaVec2 worldPos;
  CbbaVec2 lastMetricPos;

  vTaskDelay(M2T(4000));

  my_radio_low = (uint8_t)(configblockGetRadioAddress() & 0xFFu);

  if (!appIsValidNodeId(my_radio_low)) {
    while (1) {
      vTaskDelay(M2T(1000));
    }
  }

  my_agent_id = appAgentIdFromRadioLow(my_radio_low);
  startPos = startPosFor(my_radio_low);

  p2pCommInit(my_radio_low);
  Cbba_Init(&g_cbba, my_agent_id, startPos);
  Cbba_InitPeerCache(&g_peer1);
  Cbba_InitPeerCache(&g_peer2);

  while (st != ST_OFF) {
    const TickType_t nowT = xTaskGetTickCount();
    const uint32_t nowMs = (uint32_t)(nowT * portTICK_PERIOD_MS);

    bool aliveD1 = peerAliveId(NODE_ID_D1, nowT);
    bool aliveD2 = peerAliveId(NODE_ID_D2, nowT);
    bool aliveD3 = peerAliveId(NODE_ID_D3, nowT);
    bool startReadyD1 = peerStartReadyId(NODE_ID_D1);
    bool startReadyD2 = peerStartReadyId(NODE_ID_D2);
    bool startReadyD3 = peerStartReadyId(NODE_ID_D3);

    bool other_two_alive = false;
    bool all_start_ready = false;
    bool peer_takeoff_sync = false;
    bool mission_comms_enabled = false;
    bool mission_motion_enabled = false;
    bool auction_warmup_elapsed = false;

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

    if (st == ST_RUN) {
      if (distMetricValid != 0u) {
        const float dx_metric = worldPos.x_m - lastMetricPos.x_m;
        const float dy_metric = worldPos.y_m - lastMetricPos.y_m;
        const float step_metric = sqrtf((dx_metric * dx_metric) + (dy_metric * dy_metric));

        if (step_metric < 0.50f) {
          totalDistanceM += step_metric;
        }
      }

      lastMetricPos = worldPos;
      distMetricValid = 1u;
    } else {
      distMetricValid = 0u;
    }

    Cbba_SetPose(&g_cbba, worldPos);
    p2pCommSetLocalPos(worldPos.x_m, worldPos.y_m);
    p2pCommTick();

    if (my_radio_low == NODE_ID_D1) {
      aliveD1 = true;
      startReadyD1 = true;
      other_two_alive = aliveD2 && aliveD3;
    } else if (my_radio_low == NODE_ID_D2) {
      aliveD2 = true;
      startReadyD2 = true;
      other_two_alive = aliveD1 && aliveD3;
    } else {
      aliveD3 = true;
      startReadyD3 = true;
      other_two_alive = aliveD1 && aliveD2;
    }

    all_start_ready = startReadyD1 && startReadyD2 && startReadyD3;
    peer_takeoff_sync =
        (((my_radio_low != NODE_ID_D1) && peerTakeoffSyncId(NODE_ID_D1)) ||
         ((my_radio_low != NODE_ID_D2) && peerTakeoffSyncId(NODE_ID_D2)) ||
         ((my_radio_low != NODE_ID_D3) && peerTakeoffSyncId(NODE_ID_D3)));

    mission_comms_enabled =
        ((st == ST_RUN) && (run_t0 != 0) && ((nowT - run_t0) >= M2T(POST_TAKEOFF_HOLD_MS)));
    auction_warmup_elapsed =
        ((st == ST_RUN) && (run_t0 != 0) &&
         ((nowT - run_t0) >= M2T(POST_TAKEOFF_HOLD_MS + POST_TAKEOFF_AUCTION_WARMUP_MS)));

    mission_motion_enabled = auction_warmup_elapsed;

    if (mission_comms_enabled) {
      global_done_count = Cbba_GetGlobalDoneCount(&g_cbba, &g_peer1, &g_peer2);
    } else {
      global_done_count = 0u;
    }

    switch (st) {
      case ST_IDLE:
        if ((peer_takeoff_sync) && (other_two_alive)) {
          st = ST_TAKEOFF;
          st_t0 = nowT;
        } else if (all_start_ready) {
          if (stable_t0 == 0) {
            stable_t0 = nowT;
          }

#if MISSION_AUTO_START
          if ((nowT - stable_t0) > M2T(START_HOLD_MS)) {
            st = ST_TAKEOFF;
            st_t0 = nowT;
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
        }
        break;

      case ST_RUN:
        if (!other_two_alive) {
          if (peer_loss_t0 == 0) {
            peer_loss_t0 = nowT;
          }

          if ((nowT - peer_loss_t0) > M2T(PEER_LOSS_STREAK_MS)) {
            st = ST_LAND;
          }
        } else {
          peer_loss_t0 = 0;
        }

        if (global_done_count >= TASK_COUNT_RUNTIME) {
          if (g_cbba.mission_done_since_ms == 0u) {
            g_cbba.mission_done_since_ms = nowMs;
          } else if ((nowMs - g_cbba.mission_done_since_ms) >= MISSION_DONE_HOLD_MS) {
            st = ST_LAND;
          }
        } else {
          g_cbba.mission_done_since_ms = 0u;
        }
        break;

      case ST_LAND:
        if (me.position.z < 0.08f) {
          st = ST_OFF;
        }
        break;

      default:
        break;
    }

    if ((nowT - lastBeaconTx) >= M2T(1000 / BEACON_TX_HZ)) {
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
      tx.x_cm = (int16_t)((me.position.x + startPos.x_m) * 100.0f);
      tx.y_cm = (int16_t)((me.position.y + startPos.y_m) * 100.0f);
      tx.z_cm = (int16_t)(me.position.z * 100.0f);
      tx.tx_x_cm = tx.x_cm;
      tx.tx_y_cm = tx.y_cm;
      tx.total_dist_cm = cmMetricFromMeter(totalDistanceM);
      tx.done_count = g_cbba.done_count;
      tx.claim_loss_count = u8Saturate(g_cbba.claim_loss_count);
      tx.app_state = (uint8_t)st;
      tx.start_ready = (uint8_t)(((st == ST_IDLE) || (st == ST_TAKEOFF)) ? 1u : 0u);

      p2pCommSendBeacon(&tx);
    }

    {
      app_rx_event_t ev;

      while (p2pCommPollEvent(&ev)) {
        if (!mission_comms_enabled) {
          continue;
        }

        if (ev.type == MSG_CLAIM) {
          Cbba_HandleClaim(&g_cbba, &ev.u.claim, nowMs);
          updatePeerCacheFromClaimLite(my_radio_low, &ev.u.claim);
        } else if (ev.type == MSG_DONE) {
          Cbba_HandleDone(&g_cbba, &ev.u.done, nowMs);
          updatePeerCacheFromDoneLite(my_radio_low, &ev.u.done);
        } else if (ev.type == MSG_BIDVEC) {
          Cbba_HandleBidVec(&g_cbba, &ev.u.bidv, nowMs);
          updatePeerCacheFromBidVecLite(my_radio_low, &ev.u.bidv);
        }
      }
    }

    if (mission_comms_enabled) {
      Cbba_AbsorbGlobalDoneMask(&g_cbba, &g_peer1, &g_peer2, nowMs);
    }

    if (mission_comms_enabled) {
      msg_done_t doneMsg;
      msg_claim_t claimMsg;
#if ROLLING_AUCTION_ENABLE
      msg_bidvec_t bidVecMsg;
#endif

      Cbba_LocalStep(&g_cbba, nowMs);
      Cbba_MarkReachedDone(&g_cbba, nowMs);
      Cbba_AbsorbGlobalDoneMask(&g_cbba, &g_peer1, &g_peer2, nowMs);
      global_done_count = Cbba_GetGlobalDoneCount(&g_cbba, &g_peer1, &g_peer2);

      if (Cbba_MakeDoneMsg(&g_cbba, nowMs, &doneMsg)) {
        p2pCommSendDone(&doneMsg);
      }

#if ROLLING_AUCTION_ENABLE
      if (Cbba_MakeBidVecMsg(&g_cbba, nowMs, &bidVecMsg)) {
        p2pCommSendBidVec(&bidVecMsg);
      }
#endif

      if (Cbba_MakeClaimMsg(&g_cbba, nowMs, &claimMsg)) {
        p2pCommSendClaim(&claimMsg);
      }
    }

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
        if (mission_comms_enabled) {
          Cbba_AbsorbGlobalDoneMask(&g_cbba, &g_peer1, &g_peer2, nowMs);
        }

        if (!mission_motion_enabled) {
          sp.mode.x = modeAbs;
          sp.mode.y = modeAbs;
          sp.position.x = 0.0f;
          sp.position.y = 0.0f;

        } else if (timeUntilActive(nowMs, g_cbba.replan_hold_until_ms) != 0u) {
          applyIdleAvoidanceSetpoint(&sp, my_radio_low, &worldPos);
        } else if ((g_cbba.exec_task < g_cbba.task_count) &&
                   ((Cbba_GetGlobalDoneMask(&g_cbba, &g_peer1, &g_peer2) &
                     ((uint16_t)1u << g_cbba.exec_task)) == 0u)) {
          CbbaVec2 target_world;

          if (Cbba_GetExecTarget(&g_cbba, &target_world, (uint8_t*)0)) {
            const float tgt_local_x = target_world.x_m - startPos.x_m;
            const float tgt_local_y = target_world.y_m - startPos.y_m;
            const float ex = tgt_local_x - me.position.x;
            const float ey = tgt_local_y - me.position.y;
            const float dist2_local = (ex * ex) + (ey * ey);

            float vx = XY_KP * ex;
            float vy = XY_KP * ey;

            if (vx > XY_VEL_MAX) {
              vx = XY_VEL_MAX;
            }

            if (vx < -XY_VEL_MAX) {
              vx = -XY_VEL_MAX;
            }

            if (vy > XY_VEL_MAX) {
              vy = XY_VEL_MAX;
            }

            if (vy < -XY_VEL_MAX) {
              vy = -XY_VEL_MAX;
            }

            addPeerAvoidance(my_radio_low, &worldPos, &vx, &vy);

            if (dist2_local <= (GOAL_HOLD_RADIUS_M * GOAL_HOLD_RADIUS_M)) {
              float avx = 0.0f;
              float avy = 0.0f;

              addPeerAvoidance(my_radio_low, &worldPos, &avx, &avy);

              if (((avx * avx) + (avy * avy)) > 0.000001f) {
                sp.mode.x = modeVelocity;
                sp.mode.y = modeVelocity;
                sp.velocity.x = avx;
                sp.velocity.y = avy;
              } else {
                sp.mode.x = modeAbs;
                sp.mode.y = modeAbs;
                sp.position.x = tgt_local_x;
                sp.position.y = tgt_local_y;
              }
            } else {
              sp.mode.x = modeVelocity;
              sp.mode.y = modeVelocity;
              sp.velocity.x = vx;
              sp.velocity.y = vy;
            }
          } else {
            applyIdleAvoidanceSetpoint(&sp, my_radio_low, &worldPos);
          }
        } else {
          applyIdleAvoidanceSetpoint(&sp, my_radio_low, &worldPos);
        }
      }
    }

    commanderSetSetpoint(&sp, 3);
    vTaskDelay(dt);
  }

  while (1) {
    vTaskDelay(M2T(1000));
  }
}
