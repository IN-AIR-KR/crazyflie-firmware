#include "cbba_full.h"

#include <math.h>
#include <string.h>

#include "ids.h"

#define DEBUG_MODULE "CBBA"
#include "debug.h"

typedef enum {
  CBBA_ACT_LEAVE = 0,
  CBBA_ACT_UPDATE = 1,
  CBBA_ACT_RESET = 2
} CbbaResolveAction;

static float dist_m(CbbaVec2 a, CbbaVec2 b) {
  const float dx = a.x_m - b.x_m;
  const float dy = a.y_m - b.y_m;
  return sqrtf((dx * dx) + (dy * dy));
}

static int16_t cmFromMeter(float v_m) {
  return (int16_t)lrintf(v_m * 100.0f);
}

#if USE_DONE_PROTOCOL
static uint16_t taskMaskForCount(uint8_t task_count) {
  uint8_t t = 0u;
  uint16_t mask = 0u;

  for (t = 0u; t < task_count; t++) {
    mask = (uint16_t)(mask | ((uint16_t)1u << t));
  }

  return mask;
}

static uint8_t doneOwnerGet(uint32_t owner_bits, uint8_t task_id) {
  const uint8_t shift = (uint8_t)(task_id * 2u);

  if (task_id >= TASK_MAX) {
    return 0u;
  }

  return (uint8_t)((owner_bits >> shift) & 0x03u);
}

static uint32_t doneOwnerSet(uint32_t owner_bits, uint8_t task_id,
                             uint8_t owner_agent) {
  const uint8_t shift = (uint8_t)(task_id * 2u);
  const uint32_t mask = ((uint32_t)0x03u << shift);
  const uint32_t owner = ((uint32_t)(owner_agent & 0x03u) << shift);

  if (task_id >= TASK_MAX) {
    return owner_bits;
  }

  return (uint32_t)((owner_bits & (~mask)) | owner);
}

static uint16_t validatedDoneMask(uint16_t done_mask, uint32_t owner_bits,
                                  uint8_t task_count) {
  uint8_t t = 0u;
  uint16_t out = 0u;

  for (t = 0u; t < task_count; t++) {
    const uint16_t bit = (uint16_t)((uint16_t)1u << t);
    const uint8_t owner = doneOwnerGet(owner_bits, t);

    if (((done_mask & bit) != 0u) && (owner >= 1u) &&
        (owner <= AGENT_COUNT)) {
      out = (uint16_t)(out | bit);
    }
  }

  return out;
}
#endif

static uint8_t isAgentId(uint8_t agent_id) {
  return ((agent_id >= 1u) && (agent_id <= AGENT_COUNT)) ? 1u : 0u;
}

static uint8_t agentIndex(uint8_t agent_id) {
  if ((agent_id == 0u) || (agent_id > AGENT_COUNT)) {
    return 0u;
  }

  return (uint8_t)(agent_id - 1u);
}

static uint8_t normalizeWinner(uint8_t winner) {
  return (isAgentId(winner) != 0u) ? winner : 0u;
}

#if USE_DONE_PROTOCOL
static uint8_t agentMaskBit(uint8_t agent_id) {
  if (isAgentId(agent_id) == 0u) {
    return 0u;
  }

  return (uint8_t)(1u << agentIndex(agent_id));
}

static uint8_t isLostAgentLocal(const CbbaState* s, uint8_t agent_id) {
  const uint8_t bit = agentMaskBit(agent_id);

  if (bit == 0u) {
    return 0u;
  }

  return ((s->lost_agent_mask & bit) != 0u) ? 1u : 0u;
}

static uint8_t requiredDoneEchoAgentMask(const CbbaState* s) {
  uint8_t a = 0u;
  uint8_t mask = 0u;

  for (a = 1u; a <= AGENT_COUNT; a++) {
    const uint8_t bit = agentMaskBit(a);

    if (bit == 0u) {
      continue;
    }

    if (isLostAgentLocal(s, a) == 0u) {
      mask = (uint8_t)(mask | bit);
    }
  }

  return mask;
}

static void refreshPendingDoneAcks(CbbaState* s) {
  uint8_t t = 0u;
  uint16_t next_pending = 0u;
  const uint16_t active_mask = taskMaskForCount(s->task_count);
  const uint8_t required_agents = requiredDoneEchoAgentMask(s);

  s->pending_done_mask = (uint16_t)(s->pending_done_mask & active_mask);

  for (t = 0u; t < s->task_count; t++) {
    const uint16_t bit = (uint16_t)((uint16_t)1u << t);
    uint8_t a = 1u;
    uint8_t all_echoed = 1u;

    if ((s->pending_done_mask & bit) == 0u) {
      continue;
    }

    for (a = 1u; a <= AGENT_COUNT; a++) {
      const uint8_t abit = agentMaskBit(a);
      const uint8_t idx = agentIndex(a);

      if ((required_agents & abit) == 0u) {
        continue;
      }

      if ((s->peer_done_echo[idx] & bit) == 0u) {
        all_echoed = 0u;
        break;
      }
    }

    if (all_echoed == 0u) {
      next_pending = (uint16_t)(next_pending | bit);
    }
  }

  if (next_pending != s->pending_done_mask) {
    DEBUG_PRINT(
        "[DONE_ACK] agent=%u pending 0x%03X -> 0x%03X required_agents=0x%02X echo=(0x%03X,0x%03X,0x%03X)\n",
        (unsigned)s->agent_id, (unsigned)s->pending_done_mask,
        (unsigned)next_pending, (unsigned)required_agents,
        (unsigned)s->peer_done_echo[0], (unsigned)s->peer_done_echo[1],
        (unsigned)s->peer_done_echo[2]);
  }

  s->pending_done_mask = next_pending;

  if (s->pending_done_mask != 0u) {
    s->force_snapshot_tx = 1u;
  }
}
#endif

static uint8_t stampGreaterRaw(uint16_t lhs, uint16_t rhs) {
  return (lhs > rhs) ? 1u : 0u;
}

static uint8_t senderHasNewerInfoAbout(const uint16_t* sender_stamp,
                                       const uint16_t* receiver_stamp,
                                       uint8_t agent_id) {
  const uint8_t idx = agentIndex(agent_id);

  if (isAgentId(agent_id) == 0u) {
    return 0u;
  }

  return stampGreaterRaw(sender_stamp[idx], receiver_stamp[idx]);
}

static uint8_t receiverHasNewerInfoAbout(const uint16_t* receiver_stamp,
                                         const uint16_t* sender_stamp,
                                         uint8_t agent_id) {
  const uint8_t idx = agentIndex(agent_id);

  if (isAgentId(agent_id) == 0u) {
    return 0u;
  }

  return stampGreaterRaw(receiver_stamp[idx], sender_stamp[idx]);
}

static uint8_t bidDominates(uint8_t candidate_winner, int16_t candidate_bid,
                            uint8_t incumbent_winner, int16_t incumbent_bid) {
  if (candidate_bid > incumbent_bid) {
    return 1u;
  }

  if (candidate_bid < incumbent_bid) {
    return 0u;
  }

  if (candidate_winner == 0u) {
    return 0u;
  }

  if (incumbent_winner == 0u) {
    return 1u;
  }

  return (candidate_winner < incumbent_winner) ? 1u : 0u;
}

static uint8_t bidPairDominates(uint8_t candidate_winner, int16_t candidate_bid,
                                uint8_t incumbent_winner,
                                int16_t incumbent_bid) {
  return bidDominates(normalizeWinner(candidate_winner), candidate_bid,
                      normalizeWinner(incumbent_winner), incumbent_bid);
}

#if USE_DONE_PROTOCOL
static void extendReplanHold(CbbaState* s, uint32_t now_ms,
                             uint32_t hold_ms) {
  const uint32_t hold_until_ms = now_ms + hold_ms;

  if (hold_until_ms > s->replan_hold_until_ms) {
    s->replan_hold_until_ms = hold_until_ms;
  }
}

static uint8_t executionPhaseActive(const CbbaState* s, uint32_t now_ms) {
  if (s->cbba_started == 0u) {
    return 0u;
  }

  if (now_ms < s->cbba_epoch_ms) {
    return 0u;
  }

  return (((now_ms - s->cbba_epoch_ms) >= CBBA_ASSIGN_SETTLE_MS) ? 1u : 0u);
}
#endif

static uint8_t bundleContains(const CbbaState* s, uint8_t task_id) {
  uint8_t i = 0u;

  for (i = 0u; i < s->bundle_len; i++) {
    if (s->bundle[i] == task_id) {
      return 1u;
    }
  }

  return 0u;
}

static uint8_t pathFindIndex(const CbbaState* s, uint8_t task_id) {
  uint8_t i = 0u;

  for (i = 0u; i < s->path_len; i++) {
    if (s->path[i] == task_id) {
      return i;
    }
  }

  return 255u;
}

static uint8_t bundleFindIndex(const CbbaState* s, uint8_t task_id) {
  uint8_t i = 0u;

  for (i = 0u; i < s->bundle_len; i++) {
    if (s->bundle[i] == task_id) {
      return i;
    }
  }

  return 255u;
}


static uint16_t cbbaElapsedStamp(const CbbaState* s, uint32_t now_ms) {
  uint32_t elapsed_ms = 0u;
  uint32_t stamp32 = 1u;

  if (s->cbba_started == 0u) {
    return 0u;
  }

  if (now_ms >= s->cbba_epoch_ms) {
    elapsed_ms = now_ms - s->cbba_epoch_ms;
  }

  stamp32 = (elapsed_ms / CBBA_STAMP_UNIT_MS) + 1u;

  if (stamp32 > 65535u) {
    stamp32 = 65535u;
  }

  return (uint16_t)stamp32;
}

static void setOwnStamp(CbbaState* s, uint32_t now_ms) {
  const uint8_t my_idx = agentIndex(s->agent_id);
  const uint16_t st = cbbaElapsedStamp(s, now_ms);

  if (st > s->stamp[my_idx]) {
    s->stamp[my_idx] = st;
  }
}


static void pathRemove(CbbaState* s, uint8_t task_id) {
  uint8_t idx = pathFindIndex(s, task_id);
  uint8_t i = 0u;

  if (idx == 255u) {
    return;
  }

  for (i = idx; i + 1u < s->path_len; i++) {
    s->path[i] = s->path[i + 1u];
  }

  if (s->path_len > 0u) {
    s->path_len--;
  }
}

static void bundleRemove(CbbaState* s, uint8_t task_id) {
  uint8_t idx = bundleFindIndex(s, task_id);
  uint8_t i = 0u;

  if (idx == 255u) {
    return;
  }

  for (i = idx; i + 1u < s->bundle_len; i++) {
    s->bundle[i] = s->bundle[i + 1u];
  }

  if (s->bundle_len > 0u) {
    s->bundle_len--;
  }
}

#if USE_DONE_PROTOCOL
static void markTaskDone(CbbaState* s, uint8_t task_id, uint8_t local_change,
                         uint8_t owner_agent, uint32_t now_ms) {
  uint8_t changed = 0u;
  uint8_t newly_done = 0u;
  const uint16_t bit = (uint16_t)((uint16_t)1u << task_id);
  const uint8_t self_idx = agentIndex(s->agent_id);

  if (task_id >= s->task_count) {
    return;
  }

  if (isAgentId(owner_agent) == 0u) {
    return;
  }

  if ((local_change != 0u) && (s->local_exec_done[task_id] == 0u)) {
    s->local_exec_done[task_id] = 1u;
    s->local_exec_done_mask =
        (uint16_t)(s->local_exec_done_mask | bit);
    s->local_exec_done_count++;
  }

  s->pending_claim_mask = (uint16_t)(s->pending_claim_mask & (uint16_t)(~bit));

  if (s->done[task_id] == 0u) {
    s->done[task_id] = 1u;
    s->done_mask = (uint16_t)(s->done_mask | bit);
    s->done_owner_bits = doneOwnerSet(s->done_owner_bits, task_id, owner_agent);
    s->pending_done_mask = (uint16_t)(s->pending_done_mask | bit);
    s->peer_done_echo[self_idx] =
        (uint16_t)(s->peer_done_echo[self_idx] | bit);
    changed = 1u;
    newly_done = 1u;
  } else if (doneOwnerGet(s->done_owner_bits, task_id) == 0u) {
    s->done_owner_bits = doneOwnerSet(s->done_owner_bits, task_id, owner_agent);
    changed = 1u;
  }

  if ((s->winner[task_id] != 0u) || (s->bid_q[task_id] != 0)) {
    changed = 1u;
  }

  s->winner[task_id] = 0u;
  s->bid_q[task_id] = 0;
  s->ver[task_id]++;

  pathRemove(s, task_id);
  bundleRemove(s, task_id);

  if (changed != 0u) {
    s->force_snapshot_tx = 1u;
    extendReplanHold(s, now_ms, POST_DONE_ASSIGN_SETTLE_MS);
  }

  if ((local_change != 0u) && (changed != 0u)) {
    setOwnStamp(s, now_ms);
  }

  if (newly_done != 0u) {
    DEBUG_PRINT(
        "[DONE_PENDING] agent=%u task=%u pending=0x%03X echo=(0x%03X,0x%03X,0x%03X) local_change=%u\n",
        (unsigned)s->agent_id, (unsigned)task_id,
        (unsigned)s->pending_done_mask, (unsigned)s->peer_done_echo[0],
        (unsigned)s->peer_done_echo[1], (unsigned)s->peer_done_echo[2],
        (unsigned)local_change);
  }

  refreshPendingDoneAcks(s);
}

static uint8_t applyDoneMaskFirst(CbbaState* s, uint16_t done_mask,
                                  uint32_t owner_bits, uint32_t now_ms) {
  uint8_t t = 0u;
  uint8_t changed_count = 0u;
  const uint16_t active_mask = taskMaskForCount(s->task_count);
  const uint16_t masked_done = validatedDoneMask(done_mask, owner_bits,
                                                 s->task_count);

  (void)active_mask;

  for (t = 0u; t < s->task_count; t++) {
    const uint16_t bit = (uint16_t)((uint16_t)1u << t);
    const uint8_t owner = doneOwnerGet(owner_bits, t);

    if ((masked_done & bit) == 0u) {
      continue;
    }

    if (s->done[t] == 0u) {
      changed_count++;
    }

    markTaskDone(s, t, 0u, owner, now_ms);
  }

  return changed_count;
}
#endif

#if !USE_DONE_PROTOCOL
static void markTaskReachedLocalOnly(CbbaState* s, uint8_t task_id,
                                         uint32_t now_ms) {
  if (task_id >= s->task_count) {
    return;
  }

  if (s->local_exec_done[task_id] == 0u) {
    s->local_exec_done[task_id] = 1u;
    s->local_exec_done_mask =
        (uint16_t)(s->local_exec_done_mask | ((uint16_t)1u << task_id));
    s->local_exec_done_count++;
  }

  pathRemove(s, task_id);
  bundleRemove(s, task_id);
  s->replan_hold_until_ms = now_ms + REPLAN_HOLD_MS;
}
#endif

static void pathInsert(CbbaState* s, uint8_t task_id, uint8_t ins_idx) {
  uint8_t i = 0u;

  if (s->path_len >= s->bundle_limit) {
    return;
  }

  for (i = s->path_len; i > ins_idx; i--) {
    s->path[i] = s->path[i - 1u];
  }

  s->path[ins_idx] = task_id;
  s->path_len++;
}

static float pathScoreQ(const CbbaState* s, const uint8_t* path,
                        uint8_t path_len) {
  uint8_t i = 0u;
  float score_q = 0.0f;
  float elapsed_s = 0.0f;
  CbbaVec2 prev = s->self_pos;

  for (i = 0u; i < path_len; i++) {
    const uint8_t t = path[i];
    float leg_m = 0.0f;
    float task_score_q = 0.0f;

    if (t >= s->task_count) {
      continue;
    }

    leg_m = dist_m(prev, s->tasks[t].pos);

    if (CBBA_SCORING_SPEED_MPS > 0.001f) {
      elapsed_s += leg_m / CBBA_SCORING_SPEED_MPS;
    }

    task_score_q = CBBA_TASK_REWARD_Q *
                   expf(-CBBA_DISCOUNT_ALPHA_PER_SEC * elapsed_s);
    score_q += task_score_q;
    prev = s->tasks[t].pos;
  }

  return score_q;
}

static uint8_t buildInsertedPath(const CbbaState* s, uint8_t task_id,
                                 uint8_t ins_idx, uint8_t* out_path,
                                 uint8_t* out_len) {
  uint8_t i = 0u;
  uint8_t w = 0u;

  if ((out_path == (uint8_t*)0) || (out_len == (uint8_t*)0)) {
    return 0u;
  }

  if (s->path_len >= s->bundle_limit) {
    return 0u;
  }

  if (ins_idx > s->path_len) {
    return 0u;
  }

  for (i = 0u; i < ins_idx; i++) {
    out_path[w] = s->path[i];
    w++;
  }

  out_path[w] = task_id;
  w++;

  for (i = ins_idx; i < s->path_len; i++) {
    out_path[w] = s->path[i];
    w++;
  }

  *out_len = w;
  return 1u;
}

static int16_t qbid_from_score(float score_q) {
  int32_t q = 0;

  if (score_q < 0.0f) {
    score_q = 0.0f;
  }

  q = (int32_t)lrintf(score_q);

  if (q > 32767) {
    q = 32767;
  }

  if (q < 0) {
    q = 0;
  }

  return (int16_t)q;
}

static int16_t bestInsertionBid(const CbbaState* s, uint8_t task_id,
                                uint8_t* best_idx) {
  uint8_t i = 0u;
  uint8_t idx_best = 0u;
  int16_t best_bid = -32768;
  const float base_score_q = pathScoreQ(s, s->path, s->path_len);

  for (i = 0u; i <= s->path_len; i++) {
    uint8_t temp_path[TASK_MAX];
    uint8_t temp_len = 0u;
    float new_score_q = 0.0f;
    float marginal_q = 0.0f;
    int16_t bid = 0;

    memset(temp_path, 255, sizeof(temp_path));

    if (buildInsertedPath(s, task_id, i, temp_path, &temp_len) == 0u) {
      continue;
    }

    new_score_q = pathScoreQ(s, temp_path, temp_len);
    marginal_q = new_score_q - base_score_q;
    bid = qbid_from_score(marginal_q);

    if (bid > best_bid) {
      best_bid = bid;
      idx_best = i;
    }
  }

  *best_idx = idx_best;
  return best_bid;
}

static void updateLocalFp(CbbaState* s) {
  uint8_t i = 0u;
  uint32_t h = 5381u;
  uint8_t done_cnt = 0u;
  uint8_t contested = 0u;

  for (i = 0u; i < s->task_count; i++) {
    h = ((h << 5u) + h) + (uint32_t)s->done[i];
    h = ((h << 5u) + h) + (uint32_t)s->winner[i];
    h = ((h << 5u) + h) + (uint32_t)((uint16_t)s->bid_q[i]);

    if (s->done[i] != 0u) {
      done_cnt++;
    } else if (s->winner[i] == 0u) {
      contested++;
    }
  }

  for (i = 0u; i < AGENT_COUNT; i++) {
    h = ((h << 5u) + h) + (uint32_t)s->stamp[i];
  }

  s->local_fp = h;
  s->done_count = done_cnt;
  s->contested_count = contested;
  s->exec_task = (s->path_len > 0u) ? s->path[0] : 255u;
}

static void releaseSuffix(CbbaState* s, uint8_t start_idx, uint32_t now_ms) {
  uint8_t i = 0u;
  uint8_t local_changed = 0u;

  if (start_idx >= s->bundle_len) {
    return;
  }

  for (i = start_idx; i < s->bundle_len; i++) {
    const uint8_t t = s->bundle[i];

    if (t < s->task_count) {
      s->pending_claim_mask = (uint16_t)(s->pending_claim_mask & (uint16_t)(~((uint16_t)1u << t)));

      if (s->winner[t] == s->agent_id) {
        s->winner[t] = 0u;
        s->bid_q[t] = 0;
        s->ver[t]++;
        local_changed = 1u;
      }

      pathRemove(s, t);
    }
  }

  s->bundle_len = start_idx;

  if (local_changed != 0u) {
#if USE_DONE_PROTOCOL
    s->force_snapshot_tx = 1u;
    extendReplanHold(s, now_ms, POST_DONE_ASSIGN_SETTLE_MS);
#endif
    setOwnStamp(s, now_ms);
  }
}

#if USE_CONNECTIVITY_CONSTRAINT
static uint8_t isUpperStageTask(uint8_t task_id) {
  if ((task_id == 0u) || (task_id == 1u) || (task_id == 2u) ||
      (task_id == 6u) || (task_id == 7u) || (task_id == 8u)) {
    return 1u;
  }

  return 0u;
}

static uint8_t upperStageTasksRemain(const CbbaState* s) {
  uint8_t t = 0u;

  for (t = 0u; t < s->task_count; t++) {
    if ((isUpperStageTask(t) != 0u) && (s->done[t] == 0u) &&
        (s->tasks[t].active)) {
      return 1u;
    }
  }

  return 0u;
}

static CbbaVec2 predictedFrontierAnchorForAgent(uint8_t agent_id,
                                                uint8_t upper_stage) {
  CbbaVec2 p;

  if (upper_stage != 0u) {
    if (agent_id == 1u) {
      p.x_m = TASK1_X_M;
      p.y_m = TASK1_Y_M;
    } else if (agent_id == 2u) {
      p.x_m = TASK0_X_M;
      p.y_m = TASK0_Y_M;
    } else {
      p.x_m = TASK2_X_M;
      p.y_m = TASK2_Y_M;
    }
  } else {
    if (agent_id == 1u) {
      p.x_m = TASK3_X_M;
      p.y_m = TASK3_Y_M;
    } else if (agent_id == 2u) {
      p.x_m = TASK5_X_M;
      p.y_m = TASK5_Y_M;
    } else {
      p.x_m = TASK4_X_M;
      p.y_m = TASK4_Y_M;
    }
  }

  return p;
}

static uint8_t linkExists(CbbaVec2 a, CbbaVec2 b) {
  return (dist_m(a, b) <= CONNECTIVITY_RADIUS_M) ? 1u : 0u;
}

static uint8_t graphConnected3(CbbaVec2 p1, CbbaVec2 p2, CbbaVec2 p3) {
  const uint8_t l12 = linkExists(p1, p2);
  const uint8_t l13 = linkExists(p1, p3);
  const uint8_t l23 = linkExists(p2, p3);

  if (((l12 != 0u) && (l13 != 0u)) ||
      ((l12 != 0u) && (l23 != 0u)) ||
      ((l13 != 0u) && (l23 != 0u))) {
    return 1u;
  }

  return 0u;
}
#endif

static uint8_t isConnectivityFeasible(const CbbaState* s,
                                      uint8_t candidate_task) {
#if USE_CONNECTIVITY_CONSTRAINT
  CbbaVec2 p1;
  CbbaVec2 p2;
  CbbaVec2 p3;
  const uint8_t upper_stage = upperStageTasksRemain(s);

  if (candidate_task >= s->task_count) {
    return 0u;
  }

  p1 = predictedFrontierAnchorForAgent(1u, upper_stage);
  p2 = predictedFrontierAnchorForAgent(2u, upper_stage);
  p3 = predictedFrontierAnchorForAgent(3u, upper_stage);

  if (s->agent_id == 1u) {
    p1 = s->tasks[candidate_task].pos;
  } else if (s->agent_id == 2u) {
    p2 = s->tasks[candidate_task].pos;
  } else {
    p3 = s->tasks[candidate_task].pos;
  }

  return graphConnected3(p1, p2, p3);
#else
  (void)s;
  (void)candidate_task;
  return 1u;
#endif
}

static void addBundleTasks(CbbaState* s, uint32_t now_ms) {
  if (s->cbba_started == 0u) {
    return;
  }

  while (s->bundle_len < s->bundle_limit) {
    uint8_t t = 0u;
    uint8_t best_t = 255u;
    uint8_t best_ins = 0u;
    int16_t best_bid = -32768;

    for (t = 0u; t < s->task_count; t++) {
      uint8_t ins_idx = 0u;
      int16_t my_bid = 0;

      if (!s->tasks[t].active) {
        continue;
      }

      if (s->done[t] != 0u) {
        continue;
      }

      if (bundleContains(s, t) != 0u) {
        continue;
      }

#if USE_DONE_PROTOCOL
      /* During execution, do not steal a live agent's assigned unfinished
       * task. Re-auction is intentionally centered on tasks explicitly freed
       * by DONE processing or failed-winner release. Initial hover-phase CBBA
       * remains unchanged because executionPhaseActive() is false there.
       */
      if ((executionPhaseActive(s, now_ms) != 0u) &&
          (s->winner[t] != 0u) && (s->winner[t] != s->agent_id)) {
        continue;
      }
#endif

      if (isConnectivityFeasible(s, t) == 0u) {
        continue;
      }

      my_bid = bestInsertionBid(s, t, &ins_idx);

      if ((s->winner[t] == 0u) ||
          (bidDominates(s->agent_id, my_bid, s->winner[t], s->bid_q[t]) != 0u)) {
        if ((best_t == 255u) ||
            (bidDominates(s->agent_id, my_bid, s->agent_id, best_bid) != 0u)) {
          best_bid = my_bid;
          best_t = t;
          best_ins = ins_idx;
        }
      }
    }

    if (best_t == 255u) {
      break;
    }

    s->bundle[s->bundle_len] = best_t;
    s->bundle_len++;

    pathInsert(s, best_t, best_ins);

    s->winner[best_t] = s->agent_id;
    s->bid_q[best_t] = best_bid;
    s->ver[best_t]++;
#if USE_DONE_PROTOCOL
    if (executionPhaseActive(s, now_ms) != 0u) {
      s->pending_claim_mask =
          (uint16_t)(s->pending_claim_mask | ((uint16_t)1u << best_t));
      s->force_snapshot_tx = 1u;
    }
#endif
    setOwnStamp(s, now_ms);

    DEBUG_PRINT(
        "[CBBA_SELECT] agent=%u task=%u bid=%d stamp=(%u,%u,%u) conn_constraint=%u pending_claim=0x%03X\n",
        (unsigned)s->agent_id, (unsigned)best_t, (int)best_bid,
        (unsigned)s->stamp[0], (unsigned)s->stamp[1], (unsigned)s->stamp[2],
        (unsigned)USE_CONNECTIVITY_CONSTRAINT,
#if USE_DONE_PROTOCOL
        (unsigned)s->pending_claim_mask
#else
        (unsigned)0u
#endif
        );
  }
}

static CbbaResolveAction resolveTableAction(uint8_t receiver_agent,
                                            uint8_t sender_agent,
                                            uint8_t z_receiver,
                                            uint8_t z_sender,
                                            int16_t y_receiver,
                                            int16_t y_sender,
                                            const uint16_t* receiver_stamp,
                                            const uint16_t* sender_stamp) {
  const uint8_t i = receiver_agent;
  const uint8_t k = sender_agent;
  const uint8_t zi = normalizeWinner(z_receiver);
  const uint8_t zk = normalizeWinner(z_sender);

  if ((isAgentId(i) == 0u) || (isAgentId(k) == 0u) || (i == k)) {
    return CBBA_ACT_LEAVE;
  }

  /* Table I block: sender k thinks z_kj is k. */
  if (zk == k) {
    if (zi == i) {
      return (bidPairDominates(zk, y_sender, zi, y_receiver) != 0u)
                 ? CBBA_ACT_UPDATE
                 : CBBA_ACT_LEAVE;
    }

    if (zi == k) {
      return CBBA_ACT_UPDATE;
    }

    if (zi == 0u) {
      return CBBA_ACT_UPDATE;
    }

    /* Receiver thinks z_ij is m not in {i,k}. */
    if ((senderHasNewerInfoAbout(sender_stamp, receiver_stamp, zi) != 0u) ||
        (bidPairDominates(zk, y_sender, zi, y_receiver) != 0u)) {
      return CBBA_ACT_UPDATE;
    }

    return CBBA_ACT_LEAVE;
  }

  /* Table I block: sender k thinks z_kj is i. */
  if (zk == i) {
    if (zi == i) {
      return CBBA_ACT_LEAVE;
    }

    if (zi == k) {
      return CBBA_ACT_RESET;
    }

    if (zi == 0u) {
      return CBBA_ACT_LEAVE;
    }

    /* Receiver thinks z_ij is m not in {i,k}. */
    if (senderHasNewerInfoAbout(sender_stamp, receiver_stamp, zi) != 0u) {
      return CBBA_ACT_RESET;
    }

    return CBBA_ACT_LEAVE;
  }

  /* Table I block: sender k thinks z_kj is none. */
  if (zk == 0u) {
    if (zi == i) {
      return CBBA_ACT_LEAVE;
    }

    if (zi == k) {
      return CBBA_ACT_UPDATE;
    }

    if (zi == 0u) {
      return CBBA_ACT_LEAVE;
    }

    /* Receiver thinks z_ij is m not in {i,k}. */
    if (senderHasNewerInfoAbout(sender_stamp, receiver_stamp, zi) != 0u) {
      return CBBA_ACT_UPDATE;
    }

    return CBBA_ACT_LEAVE;
  }

  /* Table I block: sender k thinks z_kj is m not in {i,k}. */
  if (zi == i) {
    if ((senderHasNewerInfoAbout(sender_stamp, receiver_stamp, zk) != 0u) &&
        (bidPairDominates(zk, y_sender, zi, y_receiver) != 0u)) {
      return CBBA_ACT_UPDATE;
    }

    return CBBA_ACT_LEAVE;
  }

  if (zi == k) {
    if (senderHasNewerInfoAbout(sender_stamp, receiver_stamp, zk) != 0u) {
      return CBBA_ACT_UPDATE;
    }

    return CBBA_ACT_RESET;
  }

  if (zi == zk) {
    if (senderHasNewerInfoAbout(sender_stamp, receiver_stamp, zk) != 0u) {
      return CBBA_ACT_UPDATE;
    }

    return CBBA_ACT_LEAVE;
  }

  if (zi == 0u) {
    if (senderHasNewerInfoAbout(sender_stamp, receiver_stamp, zk) != 0u) {
      return CBBA_ACT_UPDATE;
    }

    return CBBA_ACT_LEAVE;
  }

  /* Receiver thinks z_ij is n not in {i,k,m}. */
  if ((senderHasNewerInfoAbout(sender_stamp, receiver_stamp, zk) != 0u) &&
      (senderHasNewerInfoAbout(sender_stamp, receiver_stamp, zi) != 0u)) {
    return CBBA_ACT_UPDATE;
  }

  if ((senderHasNewerInfoAbout(sender_stamp, receiver_stamp, zk) != 0u) &&
      (bidPairDominates(zk, y_sender, zi, y_receiver) != 0u)) {
    return CBBA_ACT_UPDATE;
  }

  if ((senderHasNewerInfoAbout(sender_stamp, receiver_stamp, zi) != 0u) &&
      (receiverHasNewerInfoAbout(receiver_stamp, sender_stamp, zk) != 0u)) {
    return CBBA_ACT_RESET;
  }

  return CBBA_ACT_LEAVE;
}

static void initTasks(CbbaTask* tasks) {
  uint8_t i = 0u;

  for (i = 0u; i < TASK_MAX; i++) {
    tasks[i].active = false;
    tasks[i].pos.x_m = 0.0f;
    tasks[i].pos.y_m = 0.0f;
  }

  tasks[0].active = true;
  tasks[0].pos.x_m = TASK0_X_M;
  tasks[0].pos.y_m = TASK0_Y_M;

  tasks[1].active = true;
  tasks[1].pos.x_m = TASK1_X_M;
  tasks[1].pos.y_m = TASK1_Y_M;

  tasks[2].active = true;
  tasks[2].pos.x_m = TASK2_X_M;
  tasks[2].pos.y_m = TASK2_Y_M;

  tasks[3].active = true;
  tasks[3].pos.x_m = TASK3_X_M;
  tasks[3].pos.y_m = TASK3_Y_M;

  tasks[4].active = true;
  tasks[4].pos.x_m = TASK4_X_M;
  tasks[4].pos.y_m = TASK4_Y_M;

  tasks[5].active = true;
  tasks[5].pos.x_m = TASK5_X_M;
  tasks[5].pos.y_m = TASK5_Y_M;

  tasks[6].active = true;
  tasks[6].pos.x_m = TASK6_X_M;
  tasks[6].pos.y_m = TASK6_Y_M;

  tasks[7].active = true;
  tasks[7].pos.x_m = TASK7_X_M;
  tasks[7].pos.y_m = TASK7_Y_M;

  tasks[8].active = true;
  tasks[8].pos.x_m = TASK8_X_M;
  tasks[8].pos.y_m = TASK8_Y_M;
}

void Cbba_Init(CbbaState* s, uint8_t agent_id, CbbaVec2 start_pos) {
  uint8_t i = 0u;

  memset(s, 0, sizeof(*s));

  s->agent_id = agent_id;
  s->task_count = TASK_COUNT_RUNTIME;
  s->bundle_limit = BUNDLE_LIMIT;
  s->self_pos = start_pos;
  s->exec_task = 255u;
  s->replan_hold_until_ms = 0u;
  s->mission_done_since_ms = 0u;
  s->allocation_frozen = 0u;
  s->allocation_freeze_ms = 0u;
  s->lost_agent_mask = 0u;
  s->done_mask = 0u;
  s->pending_done_mask = 0u;
  s->pending_claim_mask = 0u;
  s->local_exec_done_mask = 0u;
  s->local_exec_done_count = 0u;
  s->snap_frag_rr = 0u;
  s->cbba_started = 0u;
  s->cbba_epoch_ms = 0u;

  initTasks(s->tasks);

  for (i = 0u; i < TASK_MAX; i++) {
    s->winner[i] = 0u;
    s->bid_q[i] = 0;
    s->ver[i] = 0u;
    s->done[i] = 0u;
    s->local_exec_done[i] = 0u;
    s->done_enter_ms[i] = 0u;
  }

  for (i = 0u; i < AGENT_COUNT; i++) {
    s->stamp[i] = 0u;
    s->peer_done_echo[i] = 0u;
  }

  updateLocalFp(s);
}

void Cbba_StartMission(CbbaState* s, uint32_t now_ms) {
  uint8_t i = 0u;

  if (s->cbba_started != 0u) {
    return;
  }

  s->cbba_started = 1u;
  s->cbba_epoch_ms = now_ms;
  for (i = 0u; i < AGENT_COUNT; i++) {
    s->stamp[i] = 0u;
  }

  setOwnStamp(s, now_ms);

  addBundleTasks(s, now_ms);
  updateLocalFp(s);

  DEBUG_PRINT(
      "[CBBA_START] agent=%u epoch_ms=%lu stamp=(%u,%u,%u) path_len=%u bundle_len=%u exec=%u\n",
      (unsigned)s->agent_id, (unsigned long)s->cbba_epoch_ms,
      (unsigned)s->stamp[0], (unsigned)s->stamp[1], (unsigned)s->stamp[2],
      (unsigned)s->path_len, (unsigned)s->bundle_len, (unsigned)s->exec_task);
}

void Cbba_SetPose(CbbaState* s, CbbaVec2 pos) { s->self_pos = pos; }

void Cbba_FreezeAllocation(CbbaState* s, uint32_t now_ms) {
  if ((s->cbba_started == 0u) || (s->allocation_frozen != 0u)) {
    return;
  }

  s->allocation_frozen = 1u;
  s->allocation_freeze_ms = now_ms;
  updateLocalFp(s);

  DEBUG_PRINT(
      "[CBBA_FREEZE] agent=%u path_len=%u bundle_len=%u exec=%u stamp=(%u,%u,%u)\n",
      (unsigned)s->agent_id, (unsigned)s->path_len,
      (unsigned)s->bundle_len, (unsigned)s->exec_task,
      (unsigned)s->stamp[0], (unsigned)s->stamp[1],
      (unsigned)s->stamp[2]);
}

uint8_t Cbba_IsAllocationFrozen(const CbbaState* s) {
  return (s->allocation_frozen != 0u) ? 1u : 0u;
}

uint8_t Cbba_IsLocalExecutionFinished(const CbbaState* s) {
  if (s->allocation_frozen == 0u) {
    return 0u;
  }

  return (s->path_len == 0u) ? 1u : 0u;
}

uint8_t Cbba_GetLocalExecutionDoneCount(const CbbaState* s) {
  return s->local_exec_done_count;
}

uint8_t Cbba_HasPendingDone(const CbbaState* s) {
#if USE_DONE_PROTOCOL
  return (s->pending_done_mask != 0u) ? 1u : 0u;
#else
  (void)s;
  return 0u;
#endif
}

uint8_t Cbba_DeclareAgentLost(CbbaState* s, uint8_t lost_agent,
                              uint32_t now_ms) {
#if USE_DONE_PROTOCOL
  uint8_t t = 0u;
  uint8_t released = 0u;
  const uint8_t bit = agentMaskBit(lost_agent);

  if ((s->cbba_started == 0u) || (bit == 0u) ||
      (lost_agent == s->agent_id)) {
    return 0u;
  }

  if ((s->lost_agent_mask & bit) != 0u) {
    return 0u;
  }

  s->lost_agent_mask = (uint8_t)(s->lost_agent_mask | bit);
  refreshPendingDoneAcks(s);

  DEBUG_PRINT(
      "[FAIL_DECLARE] agent=%u lost_agent=%u lost_mask=0x%02X now=%lu\n",
      (unsigned)s->agent_id, (unsigned)lost_agent,
      (unsigned)s->lost_agent_mask, (unsigned long)now_ms);

  for (t = 0u; t < s->task_count; t++) {
    if ((s->done[t] == 0u) && (s->winner[t] == lost_agent)) {
      DEBUG_PRINT(
          "[FAIL_RELEASE] agent=%u lost_agent=%u task=%u old_bid=%d -> reset unfinished winner\n",
          (unsigned)s->agent_id, (unsigned)lost_agent, (unsigned)t,
          (int)s->bid_q[t]);

      s->winner[t] = 0u;
      s->bid_q[t] = 0;
      s->ver[t]++;
#if USE_DONE_PROTOCOL
      s->pending_claim_mask =
          (uint16_t)(s->pending_claim_mask & (uint16_t)(~((uint16_t)1u << t)));
#endif
      released++;
    }
  }

  if (released != 0u) {
    s->force_snapshot_tx = 1u;
    extendReplanHold(s, now_ms, RECOVERY_ASSIGN_SETTLE_MS);
    setOwnStamp(s, now_ms);
    refreshPendingDoneAcks(s);
    updateLocalFp(s);
  }

  return released;
#else
  (void)s;
  (void)lost_agent;
  (void)now_ms;
  return 0u;
#endif
}

void Cbba_HandleSnapshotFrag(CbbaState* s, const msg_snapshot_frag_t* m, uint32_t now_ms) {
  uint8_t k = 0u;
  uint8_t earliest_release_idx = 255u;
  uint16_t sender_stamp[AGENT_COUNT];
  const uint8_t sender_agent = appAgentIdFromRadioLow(m->src_id);
  const uint8_t sender_idx = agentIndex(sender_agent);
  const uint16_t recv_stamp = cbbaElapsedStamp(s, now_ms);
#if USE_DONE_PROTOCOL
  uint16_t incoming_done_mask = m->done_mask_global;
  uint32_t incoming_done_owner_bits = m->done_owner_bits;
#endif

  for (k = 0u; k < AGENT_COUNT; k++) {
    sender_stamp[k] = m->stamp[k];
  }

  if (isAgentId(sender_agent) != 0u) {
    sender_stamp[sender_idx] = recv_stamp;
  }

  if (s->cbba_started == 0u) {
    return;
  }

  if ((isAgentId(sender_agent) == 0u) || (sender_agent == s->agent_id)) {
    return;
  }

#if !USE_DONE_PROTOCOL
  if (s->allocation_frozen != 0u) {
    return;
  }
#endif

#if USE_DONE_PROTOCOL
  for (k = 0u; k < m->task_count_in_frag; k++) {
    const uint8_t t_mask = (uint8_t)(m->task_start_idx + k);

    if ((t_mask < s->task_count) &&
        (((m->done_mask_local >> k) & 0x01u) != 0u)) {
      incoming_done_mask =
          (uint16_t)(incoming_done_mask | ((uint16_t)1u << t_mask));

      if (doneOwnerGet(incoming_done_owner_bits, t_mask) == 0u) {
        incoming_done_owner_bits =
            doneOwnerSet(incoming_done_owner_bits, t_mask, sender_agent);
      }
    }
  }

  incoming_done_mask =
      validatedDoneMask(incoming_done_mask, incoming_done_owner_bits,
                        s->task_count);

  if (incoming_done_mask != 0u) {
    const uint8_t done_applied =
        applyDoneMaskFirst(s, incoming_done_mask, incoming_done_owner_bits, now_ms);

    s->peer_done_echo[sender_idx] =
        (uint16_t)(s->peer_done_echo[sender_idx] | incoming_done_mask);

    if (done_applied != 0u) {
      DEBUG_PRINT(
          "[DONE_RX_MASK] agent=%u sender=%u mask=0x%03X applied=%u local_mask=0x%03X pending=0x%03X\n",
          (unsigned)s->agent_id, (unsigned)sender_agent,
          (unsigned)incoming_done_mask, (unsigned)done_applied,
          (unsigned)s->done_mask, (unsigned)s->pending_done_mask);
    }

    refreshPendingDoneAcks(s);
  } else {
    refreshPendingDoneAcks(s);
  }
#endif

  for (k = 0u; k < m->task_count_in_frag; k++) {
    const uint8_t t = (uint8_t)(m->task_start_idx + k);
    uint8_t in_w = normalizeWinner(m->winner[k]);
    int16_t in_b = m->bid_q[k];
    const uint8_t in_v = m->ver[k];
    uint8_t bundle_idx = 255u;
    CbbaResolveAction action = CBBA_ACT_LEAVE;

    if (t >= s->task_count) {
      continue;
    }

#if USE_DONE_PROTOCOL
    if (s->done[t] != 0u) {
      continue;
    }

    if (isLostAgentLocal(s, in_w) != 0u) {
      DEBUG_PRINT(
          "[FAIL_IGNORE_STALE] agent=%u task=%u sender=%u stale_winner=%u -> treat as none\n",
          (unsigned)s->agent_id, (unsigned)t, (unsigned)sender_agent,
          (unsigned)in_w);
      in_w = 0u;
      in_b = 0;
    }
#endif

    action = resolveTableAction(s->agent_id, sender_agent, s->winner[t], in_w,
                                s->bid_q[t], in_b, s->stamp, sender_stamp);

    if (action == CBBA_ACT_UPDATE) {
      if ((s->winner[t] != in_w) || (s->bid_q[t] != in_b)) {
        DEBUG_PRINT(
            "[CBBA_RESOLVE] agent=%u task=%u action=UPDATE old=(w%u,b%d) new=(w%u,b%d) sender=%u\n",
            (unsigned)s->agent_id, (unsigned)t, (unsigned)s->winner[t],
            (int)s->bid_q[t], (unsigned)in_w, (int)in_b,
            (unsigned)sender_agent);
      }

      s->winner[t] = in_w;
      s->bid_q[t] = in_b;
      s->ver[t] = in_v;
#if USE_DONE_PROTOCOL
      if (in_w != s->agent_id) {
        s->pending_claim_mask =
            (uint16_t)(s->pending_claim_mask & (uint16_t)(~((uint16_t)1u << t)));
      }
#endif
    } else if (action == CBBA_ACT_RESET) {
      if ((s->winner[t] != 0u) || (s->bid_q[t] != 0)) {
        DEBUG_PRINT(
            "[CBBA_RESOLVE] agent=%u task=%u action=RESET old=(w%u,b%d) sender=%u\n",
            (unsigned)s->agent_id, (unsigned)t, (unsigned)s->winner[t],
            (int)s->bid_q[t], (unsigned)sender_agent);
      }

      s->winner[t] = 0u;
      s->bid_q[t] = 0;
      s->ver[t]++;
#if USE_DONE_PROTOCOL
      s->pending_claim_mask =
          (uint16_t)(s->pending_claim_mask & (uint16_t)(~((uint16_t)1u << t)));
#endif
    } else {
      /* Leave local y_ij and z_ij unchanged. */
    }

    bundle_idx = bundleFindIndex(s, t);
    if ((bundle_idx != 255u) && (s->winner[t] != s->agent_id)) {
      if (bundle_idx < earliest_release_idx) {
        earliest_release_idx = bundle_idx;
      }
    }
  }

  if (earliest_release_idx != 255u) {
    releaseSuffix(s, earliest_release_idx, now_ms);
  }

  if (recv_stamp > s->stamp[sender_idx]) {
    s->stamp[sender_idx] = recv_stamp;
  }

  for (k = 0u; k < AGENT_COUNT; k++) {
    if ((k != sender_idx) &&
        (stampGreaterRaw(m->stamp[k], s->stamp[k]) != 0u)) {
      s->stamp[k] = m->stamp[k];
    }
  }

  updateLocalFp(s);
}

void Cbba_LocalStep(CbbaState* s, uint32_t now_ms) {
  uint8_t i = 0u;

  if (s->cbba_started == 0u) {
    updateLocalFp(s);
    return;
  }

#if !USE_DONE_PROTOCOL
  if (s->allocation_frozen != 0u) {
    updateLocalFp(s);
    return;
  }
#endif

  for (i = 0u; i < s->bundle_len; i++) {
    const uint8_t t = s->bundle[i];

    if ((t < s->task_count) && (s->winner[t] != s->agent_id)) {
      releaseSuffix(s, i, now_ms);
      break;
    }
  }

#if USE_DONE_PROTOCOL
  /* Do not select a new task while a DONE/release update is still being
   * propagated.  pending_done_mask is cleared by peer echo ACKs, not by a
   * tuned timeout, so already-completed tasks cannot be reselected using a
   * stale DONE table.
   */
  refreshPendingDoneAcks(s);
  if ((now_ms < s->replan_hold_until_ms) || (s->pending_done_mask != 0u)) {
    updateLocalFp(s);
    return;
  }
#endif

  if (((s->path_len == 0u) || (s->bundle_len < s->bundle_limit)) &&
      (s->done_count < s->task_count)) {
    addBundleTasks(s, now_ms);
  }

  updateLocalFp(s);
}

void Cbba_MarkReachedDone(CbbaState* s, uint32_t now_ms) {
  const uint8_t t = s->exec_task;
  uint8_t i = 0u;

  if (s->cbba_started == 0u) {
    return;
  }

  if (t >= s->task_count) {
    return;
  }

#if USE_DONE_PROTOCOL
  if (s->done[t] != 0u) {
    return;
  }
#else
  if (s->allocation_frozen == 0u) {
    return;
  }

  if (s->local_exec_done[t] != 0u) {
    return;
  }
#endif

  if (dist_m(s->self_pos, s->tasks[t].pos) <= DONE_RADIUS_M) {
    if (s->done_enter_ms[t] == 0u) {
      s->done_enter_ms[t] = now_ms;
    }

    if ((now_ms - s->done_enter_ms[t]) >= DONE_DWELL_MS) {
#if USE_DONE_PROTOCOL
      markTaskDone(s, t, 1u, s->agent_id, now_ms);
#else
      markTaskReachedLocalOnly(s, t, now_ms);
#endif

      for (i = 0u; i < s->task_count; i++) {
        s->done_enter_ms[i] = 0u;
      }

      updateLocalFp(s);

#if USE_DONE_PROTOCOL
      if (s->done_count >= s->task_count) {
        if (s->mission_done_since_ms == 0u) {
          s->mission_done_since_ms = now_ms;
        }
      }

      DEBUG_PRINT(
          "[DONE_LOCAL] agent=%u task=%u done_count=%u next_exec=%u stamp=(%u,%u,%u)\n",
          (unsigned)s->agent_id, (unsigned)t, (unsigned)s->done_count,
          (unsigned)s->exec_task, (unsigned)s->stamp[0], (unsigned)s->stamp[1],
          (unsigned)s->stamp[2]);
#else
      DEBUG_PRINT(
          "[REACHED_LOCAL_ONLY] agent=%u task=%u local_exec_done=%u next_exec=%u frozen=%u\n",
          (unsigned)s->agent_id, (unsigned)t,
          (unsigned)s->local_exec_done_count, (unsigned)s->exec_task,
          (unsigned)s->allocation_frozen);
#endif
    }
  } else {
    s->done_enter_ms[t] = 0u;
  }
}

bool Cbba_MakeSnapshotFragMsg(CbbaState* s, uint32_t now_ms,
                              msg_snapshot_frag_t* out) {
  uint8_t frag_idx = 0u;
  uint8_t start_idx = 0u;
  uint8_t remain = 0u;
  uint8_t k = 0u;

  if (out == (msg_snapshot_frag_t*)0) {
    return false;
  }

  if (s->cbba_started == 0u) {
    return false;
  }

#if USE_DONE_PROTOCOL
  refreshPendingDoneAcks(s);
#endif

  if ((s->force_snapshot_tx == 0u) &&
#if USE_DONE_PROTOCOL
      (s->pending_done_mask == 0u) &&
#endif
      (s->last_snapshot_tx_ms != 0u) &&
      ((now_ms - s->last_snapshot_tx_ms) < SNAPSHOT_TX_PERIOD_MS)) {
    return false;
  }

  frag_idx = s->snap_frag_rr;

  if (frag_idx >= SNAP_FRAG_COUNT) {
    frag_idx = 0u;
  }

  start_idx = (uint8_t)(frag_idx * SNAP_FRAG_TASKS);
  remain = (uint8_t)(s->task_count - start_idx);

  setOwnStamp(s, now_ms);

  memset(out, 0, sizeof(*out));

  out->type = MSG_SNAPSHOT_FR;
  out->src_id = appNodeIdFromIndex((uint8_t)(s->agent_id - 1u));
  out->tx_id = out->src_id;
  out->seq = ++s->tx_seq;
  out->ttl = 1u;
  out->hop = 0u;
  out->tx_x_cm = cmFromMeter(s->self_pos.x_m);
  out->tx_y_cm = cmFromMeter(s->self_pos.y_m);

  out->frag_idx = frag_idx;
  out->frag_count = SNAP_FRAG_COUNT;
  out->task_start_idx = start_idx;
  out->task_count_total = s->task_count;
  out->task_count_in_frag =
      (remain > SNAP_FRAG_TASKS) ? SNAP_FRAG_TASKS : remain;
  out->exec_task = s->exec_task;

  for (k = 0u; k < AGENT_COUNT; k++) {
    out->stamp[k] = s->stamp[k];
  }

#if USE_DONE_PROTOCOL
  out->done_mask_global = (uint16_t)(s->done_mask & taskMaskForCount(s->task_count));
  out->done_owner_bits = s->done_owner_bits;
#else
  out->done_mask_global = 0u;
  out->done_owner_bits = 0u;
#endif

  for (k = 0u; k < out->task_count_in_frag; k++) {
    const uint8_t t = (uint8_t)(start_idx + k);

#if USE_DONE_PROTOCOL
    out->done_mask_local |= (uint8_t)((s->done[t] & 0x01u) << k);
#else
    out->done_mask_local = 0u;
#endif
    out->winner[k] = s->winner[t];
    out->bid_q[k] = s->bid_q[t];
    out->ver[k] = s->ver[t];
  }

  s->snap_frag_rr++;

  if (s->snap_frag_rr >= SNAP_FRAG_COUNT) {
    s->snap_frag_rr = 0u;
  }

  s->last_snapshot_tx_ms = now_ms;
  s->force_snapshot_tx = 0u;
#if USE_DONE_PROTOCOL
  if (s->pending_done_mask != 0u) {
    s->force_snapshot_tx = 1u;
  }
#endif
  return true;
}

void Cbba_InitPeerCache(PeerSnapshotCache* c) {
  memset(c, 0, sizeof(*c));

  c->src_id = 0u;
  c->exec_task = 255u;
}

void Cbba_UpdatePeerCacheFromFrag(PeerSnapshotCache* c,
                                  const msg_snapshot_frag_t* m) {
  uint8_t k = 0u;
#if USE_DONE_PROTOCOL
  uint8_t t_all = 0u;
  const uint16_t global_done =
      validatedDoneMask(m->done_mask_global, m->done_owner_bits,
                        m->task_count_total);
#endif

  c->valid = 1u;
  c->src_id = m->src_id;
  c->frag_count = m->frag_count;
  c->task_count_total = m->task_count_total;
  c->exec_task = m->exec_task;
  c->got_mask |= (uint8_t)(1u << m->frag_idx);

  for (k = 0u; k < AGENT_COUNT; k++) {
    c->stamp[k] = m->stamp[k];
  }

#if USE_DONE_PROTOCOL
  c->done_owner_bits = m->done_owner_bits;

  for (t_all = 0u; t_all < TASK_MAX; t_all++) {
    const uint16_t bit = (uint16_t)((uint16_t)1u << t_all);

    if ((global_done & bit) != 0u) {
      c->done[t_all] = 1u;
      c->done_mask = (uint16_t)(c->done_mask | bit);
      c->winner[t_all] = 0u;
      c->bid_q[t_all] = 0;
    }
  }
#endif

  for (k = 0u; k < m->task_count_in_frag; k++) {
    const uint8_t t = (uint8_t)(m->task_start_idx + k);

    if (t >= TASK_MAX) {
      continue;
    }

#if USE_DONE_PROTOCOL
    if (((m->done_mask_local >> k) & 0x01u) != 0u) {
      uint8_t owner = doneOwnerGet(c->done_owner_bits, t);

      if (owner == 0u) {
        owner = appAgentIdFromRadioLow(m->src_id);
        c->done_owner_bits = doneOwnerSet(c->done_owner_bits, t, owner);
      }

      if (isAgentId(owner) != 0u) {
        c->done[t] = 1u;
        c->done_mask = (uint16_t)(c->done_mask | ((uint16_t)1u << t));
        c->winner[t] = 0u;
        c->bid_q[t] = 0;
        c->ver[t] = m->ver[k];
      }
    } else if (c->done[t] == 0u) {
      c->winner[t] = normalizeWinner(m->winner[k]);
      c->bid_q[t] = m->bid_q[k];
      c->ver[t] = m->ver[k];
    }
#else
    c->winner[t] = normalizeWinner(m->winner[k]);
    c->bid_q[t] = m->bid_q[k];
    c->ver[t] = m->ver[k];
#endif
  }
}

static uint32_t fpFromArrays(const uint8_t* done, const uint8_t* winner,
                             const int16_t* bid_q, const uint16_t* stamp,
                             uint8_t task_count) {
  uint8_t i = 0u;
  uint32_t h = 5381u;

  for (i = 0u; i < task_count; i++) {
    h = ((h << 5u) + h) + (uint32_t)done[i];
    h = ((h << 5u) + h) + (uint32_t)winner[i];
    h = ((h << 5u) + h) + (uint32_t)((uint16_t)bid_q[i]);
  }

  for (i = 0u; i < AGENT_COUNT; i++) {
    h = ((h << 5u) + h) + (uint32_t)stamp[i];
  }

  return h;
}

void Cbba_GetObserverMetrics(const CbbaState* self,
                             const PeerSnapshotCache* peer1,
                             const PeerSnapshotCache* peer2,
                             CbbaObserverMetrics* out) {
  uint8_t t = 0u;
  uint8_t need_mask = 0u;
  const uint8_t task_count = self->task_count;

  memset(out, 0, sizeof(*out));

  out->exec_shadow[0] = self->exec_task;
  out->exec_shadow[1] = peer1->exec_task;
  out->exec_shadow[2] = peer2->exec_task;

  if ((peer1->valid == 0u) || (peer2->valid == 0u)) {
    out->all_known = 0u;
    out->fp_shadow[0] = fpFromArrays(self->done, self->winner, self->bid_q,
                                     self->stamp, task_count);
    return;
  }

  need_mask = (uint8_t)((1u << SNAP_FRAG_COUNT) - 1u);

  if (((peer1->got_mask & need_mask) != need_mask) ||
      ((peer2->got_mask & need_mask) != need_mask)) {
    out->all_known = 0u;
    out->fp_shadow[0] = fpFromArrays(self->done, self->winner, self->bid_q,
                                     self->stamp, task_count);
    out->fp_shadow[1] = fpFromArrays(peer1->done, peer1->winner, peer1->bid_q,
                                     peer1->stamp, task_count);
    out->fp_shadow[2] = fpFromArrays(peer2->done, peer2->winner, peer2->bid_q,
                                     peer2->stamp, task_count);
    return;
  }

  out->all_known = 1u;
  out->fp_shadow[0] = fpFromArrays(self->done, self->winner, self->bid_q,
                                   self->stamp, task_count);
  out->fp_shadow[1] = fpFromArrays(peer1->done, peer1->winner, peer1->bid_q,
                                   peer1->stamp, task_count);
  out->fp_shadow[2] = fpFromArrays(peer2->done, peer2->winner, peer2->bid_q,
                                   peer2->stamp, task_count);

  for (t = 0u; t < task_count; t++) {
    const uint8_t w0 = self->winner[t];
    const uint8_t w1 = peer1->winner[t];
    const uint8_t w2 = peer2->winner[t];

    if ((w0 == w1) && (w1 == w2)) {
      out->equal_winner_tasks++;
    } else {
      out->contested_tasks++;
    }
  }

  out->winner_conv = (out->equal_winner_tasks == task_count) ? 1u : 0u;
  out->fp_conv = ((out->fp_shadow[0] == out->fp_shadow[1]) &&
                  (out->fp_shadow[1] == out->fp_shadow[2]))
                     ? 1u
                     : 0u;
}

uint16_t Cbba_GetGlobalDoneMask(const CbbaState* self,
                                const PeerSnapshotCache* peer1,
                                const PeerSnapshotCache* peer2) {
#if USE_DONE_PROTOCOL
  const uint16_t active_mask = taskMaskForCount(self->task_count);

  /* Mission completion must be based on the local validated DONE consensus
   * state, not a raw OR of peer caches.  Peer DONE information becomes
   * authoritative only after Cbba_HandleSnapshotFrag() validates its compact
   * owner/provenance field and commits it into self->done[].
   */
  (void)peer1;
  (void)peer2;
  return (uint16_t)(self->done_mask & active_mask);
#else
  (void)self;
  (void)peer1;
  (void)peer2;
  return 0u;
#endif
}

uint8_t Cbba_GetGlobalDoneCount(const CbbaState* self,
                                const PeerSnapshotCache* peer1,
                                const PeerSnapshotCache* peer2) {
  uint8_t t = 0u;
  uint8_t cnt = 0u;
  const uint16_t mask = Cbba_GetGlobalDoneMask(self, peer1, peer2);

  for (t = 0u; t < self->task_count; t++) {
    if ((mask & ((uint16_t)1u << t)) != 0u) {
      cnt++;
    }
  }

  return cnt;
}


#if USE_DONE_PROTOCOL
static uint8_t peerCacheHasTaskInfo(const PeerSnapshotCache* c, uint8_t task_id) {
  const uint8_t frag_idx = (uint8_t)(task_id / SNAP_FRAG_TASKS);

  if ((c == (const PeerSnapshotCache*)0) || (c->valid == 0u)) {
    return 0u;
  }

  if (frag_idx >= SNAP_FRAG_COUNT) {
    return 0u;
  }

  return ((c->got_mask & ((uint8_t)1u << frag_idx)) != 0u) ? 1u : 0u;
}

static uint8_t peerEchoesWinnerForTask(const PeerSnapshotCache* c,
                                       uint8_t task_id,
                                       uint8_t winner_agent) {
  if (peerCacheHasTaskInfo(c, task_id) == 0u) {
    return 0u;
  }

  if (c->done[task_id] != 0u) {
    return 0u;
  }

  return (c->winner[task_id] == winner_agent) ? 1u : 0u;
}

static const PeerSnapshotCache* findPeerCacheByAgent(
    const PeerSnapshotCache* peer1, const PeerSnapshotCache* peer2,
    uint8_t agent_id) {
  const uint8_t radio_id = appNodeIdFromIndex(agentIndex(agent_id));

  if ((peer1 != (const PeerSnapshotCache*)0) && (peer1->valid != 0u) &&
      (peer1->src_id == radio_id)) {
    return peer1;
  }

  if ((peer2 != (const PeerSnapshotCache*)0) && (peer2->valid != 0u) &&
      (peer2->src_id == radio_id)) {
    return peer2;
  }

  return (const PeerSnapshotCache*)0;
}
#endif

uint8_t Cbba_CanMoveToExec(const CbbaState* self,
                           const PeerSnapshotCache* peer1,
                           const PeerSnapshotCache* peer2) {
#if USE_DONE_PROTOCOL
  uint8_t a = 1u;
  const uint8_t t = self->exec_task;
  const uint16_t bit = (uint16_t)((uint16_t)1u << t);

  if (t >= self->task_count) {
    return 0u;
  }

  if (self->done[t] != 0u) {
    return 0u;
  }

  if ((self->pending_claim_mask & bit) == 0u) {
    return 1u;
  }

  if (self->winner[t] != self->agent_id) {
    return 0u;
  }

  for (a = 1u; a <= AGENT_COUNT; a++) {
    const PeerSnapshotCache* pc = (const PeerSnapshotCache*)0;

    if (a == self->agent_id) {
      continue;
    }

    if (isLostAgentLocal(self, a) != 0u) {
      continue;
    }

    pc = findPeerCacheByAgent(peer1, peer2, a);
    if (pc == (const PeerSnapshotCache*)0) {
      return 0u;
    }

    if (peerEchoesWinnerForTask(pc, t, self->agent_id) == 0u) {
      return 0u;
    }
  }

  return 1u;
#else
  (void)self;
  (void)peer1;
  (void)peer2;
  return 1u;
#endif
}

void Cbba_DebugPrintTables(const char* tag, const CbbaState* self,
                           const PeerSnapshotCache* peer1,
                           const PeerSnapshotCache* peer2) {
  (void)tag;
  (void)self;
  (void)peer1;
  (void)peer2;

  uint8_t t = 0u;
  uint8_t i = 0u;

  DEBUG_PRINT(
      "[CBBA_DUMP] tag=%s agent=%u started=%u exec=%u done_count=%u task_count=%u "
      "done_mask=0x%03X local_exec=%u frozen=%u path_len=%u bundle_len=%u stamp=(%u,%u,%u) "
      "peer1_id=0x%02X peer2_id=0x%02X\n",
      tag, (unsigned)self->agent_id, (unsigned)self->cbba_started,
      (unsigned)self->exec_task, (unsigned)self->done_count,
      (unsigned)self->task_count, (unsigned)self->done_mask,
      (unsigned)self->local_exec_done_count,
      (unsigned)self->allocation_frozen, (unsigned)self->path_len,
      (unsigned)self->bundle_len,
      (unsigned)self->stamp[0], (unsigned)self->stamp[1],
      (unsigned)self->stamp[2], (unsigned)peer1->src_id,
      (unsigned)peer2->src_id);

  for (t = 0u; t < self->task_count; t++) {
    DEBUG_PRINT(
        "[CBBA_TASK_SELF] t=%u act=%u done=%u winner=%u bid=%d ver=%u "
        "pos=(%.2f,%.2f) in_path=%u in_bundle=%u\n",
        (unsigned)t, (unsigned)(self->tasks[t].active ? 1u : 0u),
        (unsigned)self->done[t], (unsigned)self->winner[t],
        (int)self->bid_q[t], (unsigned)self->ver[t],
        (double)self->tasks[t].pos.x_m, (double)self->tasks[t].pos.y_m,
        (unsigned)((pathFindIndex(self, t) != 255u) ? 1u : 0u),
        (unsigned)((bundleFindIndex(self, t) != 255u) ? 1u : 0u));

    DEBUG_PRINT(
        "[CBBA_TASK_PEER] t=%u "
        "P1(id=0x%02X,valid=%u,done=%u,winner=%u,bid=%d,ver=%u,stamp=(%u,%u,%u)) "
        "P2(id=0x%02X,valid=%u,done=%u,winner=%u,bid=%d,ver=%u,stamp=(%u,%u,%u))\n",
        (unsigned)t, (unsigned)peer1->src_id, (unsigned)peer1->valid,
        (unsigned)peer1->done[t], (unsigned)peer1->winner[t],
        (int)peer1->bid_q[t], (unsigned)peer1->ver[t],
        (unsigned)peer1->stamp[0], (unsigned)peer1->stamp[1],
        (unsigned)peer1->stamp[2], (unsigned)peer2->src_id,
        (unsigned)peer2->valid, (unsigned)peer2->done[t],
        (unsigned)peer2->winner[t], (int)peer2->bid_q[t],
        (unsigned)peer2->ver[t], (unsigned)peer2->stamp[0],
        (unsigned)peer2->stamp[1], (unsigned)peer2->stamp[2]);
  }

  for (i = 0u; i < TASK_MAX; i++) {
    const uint8_t path_v = (i < self->path_len) ? self->path[i] : 255u;
    const uint8_t bundle_v = (i < self->bundle_len) ? self->bundle[i] : 255u;

    (void)path_v;
    (void)bundle_v;

    DEBUG_PRINT("[CBBA_PATH_BUNDLE] idx=%u path=%u bundle=%u\n", (unsigned)i,
                (unsigned)path_v, (unsigned)bundle_v);
  }
}
