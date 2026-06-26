#include "cbba_full.h"

#include <math.h>
#include <string.h>

#include "ids.h"

#define BIDVEC_INVALID_Q (-32768)

#if !ROLLING_AUCTION_ENABLE
static uint16_t qabs_i16(int16_t v) {
  return (uint16_t)((v < 0) ? -v : v);
}
#endif

static float dist_m(CbbaVec2 a, CbbaVec2 b) {
  const float dx = a.x_m - b.x_m;
  const float dy = a.y_m - b.y_m;
  return sqrtf((dx * dx) + (dy * dy));
}

static int16_t clampBidQ(int32_t q) {
  if (q > 32767) {
    q = 32767;
  }

  if (q < -32768) {
    q = -32768;
  }

  return (int16_t)q;
}

static int16_t qbid_from_task(const CbbaTask* task, CbbaVec2 base_pos) {
  float cost_m = 0.0f;

  if (task == (const CbbaTask*)0) {
    return 0;
  }

  cost_m = dist_m(base_pos, task->pos) + dist_m(task->pos, task->exit_pos);

  return clampBidQ((int32_t)task->value_q -
                   (int32_t)lrintf(cost_m * (float)TASK_DISTANCE_BID_Q_PER_M));
}

static int16_t cmFromMeter(float v_m) {
  return (int16_t)lrintf(v_m * 100.0f);
}

static uint16_t taskMaskForCount(uint8_t task_count) {
  uint8_t t = 0u;
  uint16_t mask = 0u;

  for (t = 0u; t < task_count; t++) {
    mask = (uint16_t)(mask | ((uint16_t)1u << t));
  }

  return mask;
}

static uint8_t activeTaskLimit(const CbbaState* s) {
  return s->task_count;
}

static uint8_t taskIsReleased(const CbbaState* s, uint8_t task_id) {
  if (task_id >= s->task_count) {
    return 0u;
  }

  if (!s->tasks[task_id].active) {
    return 0u;
  }

  return (task_id < activeTaskLimit(s)) ? 1u : 0u;
}

#if DEMO_AGENT_TASK_CAP_ENABLE
static uint8_t demoTaskCapForAgentIndex(uint8_t agent_idx) {
  if (agent_idx == 0u) {
    return DEMO_D1_TASK_CAP;
  }

  if (agent_idx == 1u) {
    return DEMO_D2_TASK_CAP;
  }

  if (agent_idx == 2u) {
    return DEMO_D3_TASK_CAP;
  }

  return BUNDLE_LIMIT;
}

static uint8_t demoRemainingTaskCapForSelf(const CbbaState* s) {
  const uint8_t self_idx = (uint8_t)(s->agent_id - 1u);
  const uint8_t cap = demoTaskCapForAgentIndex(self_idx);

  if (s->local_done_count >= cap) {
    return 0u;
  }

  return (uint8_t)(cap - s->local_done_count);
}

static uint8_t demoRemainingTaskCapForAgent(const CbbaState* s, uint8_t agent_idx,
                                            uint8_t self_idx) {
  const uint8_t cap = demoTaskCapForAgentIndex(agent_idx);

  if (agent_idx == self_idx) {
    if (s->local_done_count >= cap) {
      return 0u;
    }

    return (uint8_t)(cap - s->local_done_count);
  }

  return cap;
}

static uint8_t demoSelfTaskCapReached(const CbbaState* s) {
  return (demoRemainingTaskCapForSelf(s) == 0u) ? 1u : 0u;
}
#endif

static void updateLocalFp(CbbaState* s);
static void pruneInvalidAssignments(CbbaState* s);

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

static uint8_t taskInPath(const CbbaState* s, uint8_t task_id) {
  return (pathFindIndex(s, task_id) != 255u) ? 1u : 0u;
}

static uint8_t taskInBundle(const CbbaState* s, uint8_t task_id) {
  return (bundleFindIndex(s, task_id) != 255u) ? 1u : 0u;
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

static void markTaskDone(CbbaState* s, uint8_t task_id, uint8_t force_ver_inc) {
  if (task_id >= s->task_count) {
    return;
  }

  if (s->done[task_id] == 0u) {
    s->done[task_id] = 1u;
    s->done_mask = (uint16_t)(s->done_mask | ((uint16_t)1u << task_id));
  }

  s->winner[task_id] = 0u;
  s->bid_q[task_id] = 0;
  s->winner_rx_ms[task_id] = 0u;
  s->done_rr = task_id;

  if (force_ver_inc != 0u) {
    s->ver[task_id]++;
  }

  pathRemove(s, task_id);
  bundleRemove(s, task_id);
  s->lane_entered[task_id] = 0u;
}

static uint8_t absorbDoneMask(CbbaState* s, uint16_t done_mask) {
  uint8_t i = 0u;
  uint8_t changed = 0u;

  for (i = 0u; i < s->task_count; i++) {
    const uint8_t in_done_mask = ((done_mask & ((uint16_t)1u << i)) != 0u) ? 1u : 0u;
    const uint8_t has_stale_local_ref =
        ((taskInPath(s, i) != 0u) || (taskInBundle(s, i) != 0u) || (s->winner[i] == s->agent_id) ||
         (s->exec_task == i))
            ? 1u
            : 0u;

    if (in_done_mask == 0u) {
      continue;
    }

    if ((s->done[i] == 0u) || (has_stale_local_ref != 0u)) {
      if (s->ver[i] == 0u) {
        s->ver[i] = 1u;
      }
      markTaskDone(s, i, 0u);
      changed = 1u;
    }
  }

  if (changed != 0u) {
    pruneInvalidAssignments(s);
    updateLocalFp(s);
  }

  return changed;
}

static uint8_t peerReservationWins(const CbbaState* s, uint8_t task_id, uint8_t peer_agent,
                                   int16_t peer_bid) {
  const uint8_t current = s->winner[task_id];

  if (current == 0u) {
    return 1u;
  }

  if (current == peer_agent) {
    return 1u;
  }

  /* Reservation-mode conflict resolution. Prefer the higher distance-based
   * bid, then use agent id only as a deterministic tie-break. Mesh relay
   * exposes hidden D1/D3 duplicate reservations earlier than direct P2P.
   */
  if (peer_bid > s->bid_q[task_id]) {
    return 1u;
  }

  if (peer_bid < s->bid_q[task_id]) {
    return 0u;
  }

  if (current == s->agent_id) {
    return (peer_agent < s->agent_id) ? 1u : 0u;
  }

  return (peer_agent < current) ? 1u : 0u;
}

static uint16_t reservationMask(const CbbaState* s) {
  uint8_t t = 0u;
  uint16_t mask = 0u;

  for (t = 0u; t < s->task_count; t++) {
    if ((s->done[t] == 0u) && (s->winner[t] == s->agent_id)) {
      mask = (uint16_t)(mask | ((uint16_t)1u << t));
    }
  }

  return mask;
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
    h = ((h << 5u) + h) + (uint32_t)s->ver[i];

    if (s->done[i] != 0u) {
      done_cnt++;
    } else if ((taskIsReleased(s, i) != 0u) && (s->winner[i] == 0u)) {
      contested++;
    }
  }

  s->local_fp = h;
  s->done_count = done_cnt;
  s->contested_count = contested;
  s->exec_task = (s->path_len > 0u) ? s->path[0] : 255u;
}

static void releaseSuffix(CbbaState* s, uint8_t start_idx) {
  uint8_t i = 0u;

  if (start_idx >= s->bundle_len) {
    return;
  }

  for (i = start_idx; i < s->bundle_len; i++) {
    const uint8_t t = s->bundle[i];

    if (t < s->task_count) {
      if (s->winner[t] == s->agent_id) {
        s->winner[t] = 0u;
        s->bid_q[t] = 0;
        s->winner_rx_ms[t] = 0u;
        s->ver[t]++;
      }

      pathRemove(s, t);
    }
  }

  s->bundle_len = start_idx;
}

static void pruneInvalidAssignments(CbbaState* s) {
  uint8_t i = 0u;

  for (i = 0u; i < s->bundle_len;) {
    const uint8_t t = s->bundle[i];

    if ((t >= s->task_count) || (taskIsReleased(s, t) == 0u) || (s->done[t] != 0u) ||
        (s->winner[t] != s->agent_id)) {
      bundleRemove(s, t);
      pathRemove(s, t);
      s->lane_entered[t] = 0u;
    } else {
      i++;
    }
  }

  for (i = 0u; i < s->path_len;) {
    const uint8_t t = s->path[i];

    if ((t >= s->task_count) || (taskIsReleased(s, t) == 0u) || (s->done[t] != 0u) ||
        (s->winner[t] != s->agent_id)) {
      pathRemove(s, t);
      s->lane_entered[t] = 0u;
    } else {
      i++;
    }
  }
}

static void clearPeerReservationsOutsideMask(CbbaState* s, uint8_t peer_agent, uint16_t keep_mask) {
  uint8_t t = 0u;

  for (t = 0u; t < s->task_count; t++) {
    if ((s->done[t] == 0u) && (s->winner[t] == peer_agent) &&
        ((keep_mask & ((uint16_t)1u << t)) == 0u)) {
      s->winner[t] = 0u;
      s->bid_q[t] = 0;
      s->winner_rx_ms[t] = 0u;
    }
  }
}

static void setClaimBurst(CbbaState* s) {
  s->claim_burst_left = CLAIM_BURST_COUNT;
  s->last_claim_tx_ms = 0u;
}

static void setAssignmentBroadcastBurst(CbbaState* s) {
  s->last_bidvec_tx_ms = 0u;
  setClaimBurst(s);
}

#if !ROLLING_AUCTION_ENABLE
static void pruneStalePeerReservations(CbbaState* s, uint32_t now_ms) {
  uint8_t t = 0u;
  uint8_t changed = 0u;

  for (t = 0u; t < s->task_count; t++) {
    if ((s->done[t] == 0u) && (s->winner[t] != 0u) && (s->winner[t] != s->agent_id) &&
        (s->winner_rx_ms[t] != 0u) && (now_ms >= s->winner_rx_ms[t]) &&
        ((now_ms - s->winner_rx_ms[t]) > CLAIM_STALE_MS)) {
      s->winner[t] = 0u;
      s->bid_q[t] = 0;
      s->winner_rx_ms[t] = 0u;
      changed = 1u;
    }
  }

  if (changed != 0u) {
    pruneInvalidAssignments(s);
  }
}

static void refreshLocalClaimBids(CbbaState* s, uint32_t now_ms) {
  uint8_t i = 0u;
  uint8_t changed = 0u;

  if ((s->last_bid_refresh_ms != 0u) &&
      ((now_ms - s->last_bid_refresh_ms) < CLAIM_BID_REFRESH_MS)) {
    return;
  }

  s->last_bid_refresh_ms = now_ms;

  for (i = 0u; i < s->bundle_len; i++) {
    const uint8_t t = s->bundle[i];
    int16_t new_bid = 0;
    int16_t old_bid = 0;
    int16_t delta = 0;

    if ((t >= s->task_count) || (s->done[t] != 0u) || (s->winner[t] != s->agent_id)) {
      continue;
    }

    old_bid = s->bid_q[t];
    new_bid = qbid_from_task(&s->tasks[t], s->self_pos);
    delta = (int16_t)(new_bid - old_bid);

    if (qabs_i16(delta) >= CLAIM_BID_REFRESH_DELTA_Q) {
      s->bid_q[t] = new_bid;
      s->ver[t]++;
      changed = 1u;

    }
  }

  if (changed != 0u) {
    setClaimBurst(s);
    updateLocalFp(s);
  }
}
#endif

static void addBundleTasks(CbbaState* s) {
#if DEMO_AGENT_TASK_CAP_ENABLE
  const uint8_t self_cap = demoRemainingTaskCapForSelf(s);

  if (self_cap == 0u) {
    return;
  }
#endif

  while (s->bundle_len < s->bundle_limit) {
    uint8_t t = 0u;
    uint8_t best_t = 255u;
    int16_t best_bid = -32768;
    CbbaVec2 base_pos = s->self_pos;

#if DEMO_AGENT_TASK_CAP_ENABLE
    if (s->bundle_len >= self_cap) {
      return;
    }
#endif

    if (s->path_len > 0u) {
      const uint8_t last_t = s->path[s->path_len - 1u];

      if (last_t < s->task_count) {
        base_pos = s->tasks[last_t].pos;
      }
    }

    for (t = 0u; t < s->task_count; t++) {
      int16_t my_bid = 0;

      if (taskIsReleased(s, t) == 0u) {
        continue;
      }

      if (s->done[t] != 0u) {
        continue;
      }

      if (bundleContains(s, t) != 0u) {
        continue;
      }

      /* Simple distributed reservation:
       * choose the closest task that is not done and not already reserved by a
       * heard peer. With BUNDLE_LIMIT=2 this reserves the current response
       * point and the next likely response point, so mesh-relayed reservations
       * can prevent duplicate approaches throughout the flight instead of only
       * at the first task.
       */
      if ((s->winner[t] != 0u) && (s->winner[t] != s->agent_id)) {
        continue;
      }

      my_bid = qbid_from_task(&s->tasks[t], base_pos);

      if (my_bid > best_bid) {
        best_bid = my_bid;
        best_t = t;
      }
    }

    if (best_t == 255u) {
      return;
    }

    s->bundle[s->bundle_len] = best_t;
    s->bundle_len++;
    s->path[s->path_len] = best_t;
    s->path_len++;

    s->winner[best_t] = s->agent_id;
    s->bid_q[best_t] = best_bid;
    s->winner_rx_ms[best_t] = 0u;
    s->ver[best_t]++;
    setClaimBurst(s);
  }
}

static uint8_t bidRowFresh(const CbbaState* s, uint8_t agent_idx, uint32_t now_ms) {
  const uint8_t self_idx = (uint8_t)(s->agent_id - 1u);

  if (agent_idx == self_idx) {
    return 1u;
  }

  if ((agent_idx >= AGENT_COUNT) || (s->peer_bid_valid[agent_idx] == 0u) ||
      (s->peer_bid_rx_ms[agent_idx] == 0u)) {
    return 0u;
  }

  if (now_ms < s->peer_bid_rx_ms[agent_idx]) {
    return 1u;
  }

  if ((now_ms - s->peer_bid_rx_ms[agent_idx]) <= BIDVEC_STALE_MS) {
    return 1u;
  }

  return 0u;
}

static int16_t bidForAgentTask(const CbbaState* s, uint8_t agent_idx, uint8_t task_id) {
  const uint8_t self_idx = (uint8_t)(s->agent_id - 1u);
  const uint16_t task_bit = (uint16_t)((uint16_t)1u << task_id);

  if ((task_id >= s->task_count) || (taskIsReleased(s, task_id) == 0u) ||
      (s->done[task_id] != 0u)) {
    return BIDVEC_INVALID_Q;
  }

  if (agent_idx == self_idx) {
#if DEMO_AGENT_TASK_CAP_ENABLE
    if (demoSelfTaskCapReached(s) != 0u) {
      return BIDVEC_INVALID_Q;
    }
#endif

    return qbid_from_task(&s->tasks[task_id], s->self_pos);
  }

  if ((agent_idx >= AGENT_COUNT) || (s->peer_bid_valid[agent_idx] == 0u) ||
      ((s->peer_done_mask[agent_idx] & task_bit) != 0u) || (s->done[task_id] != 0u)) {
    return BIDVEC_INVALID_Q;
  }

  return s->peer_bid_q[agent_idx][task_id];
}

static uint8_t candidateBetter(int16_t cand_bid, uint8_t cand_agent, uint8_t cand_task,
                               uint8_t has_best, int16_t best_bid, uint8_t best_agent,
                               uint8_t best_task) {
  if (has_best == 0u) {
    return 1u;
  }

  if (cand_bid > best_bid) {
    return 1u;
  }

  if (cand_bid < best_bid) {
    return 0u;
  }

  if (cand_agent < best_agent) {
    return 1u;
  }

  if (cand_agent > best_agent) {
    return 0u;
  }

  return (cand_task < best_task) ? 1u : 0u;
}

static uint8_t execTaskForAgent(const CbbaState* s, uint8_t agent_idx, uint8_t self_idx) {
  if (agent_idx == self_idx) {
    return s->exec_task;
  }

  if (agent_idx < AGENT_COUNT) {
    return s->peer_exec_task[agent_idx];
  }

  return 255u;
}

static void rollingAuctionAssign(CbbaState* s, uint32_t now_ms) {
  uint8_t agent_available[AGENT_COUNT];
  uint8_t agent_task_limit[AGENT_COUNT];
  uint8_t agent_task_count[AGENT_COUNT];
  uint8_t agent_task_order[AGENT_COUNT][TASK_MAX];
  uint8_t task_owner[TASK_MAX];
  int16_t task_bid[TASK_MAX];
#if ACTIVE_EXEC_RESERVATION_LOCK_ENABLE
  uint8_t reservation_owner[TASK_MAX];
  int16_t reservation_bid[TASK_MAX];
#endif
  const uint8_t old_exec = s->exec_task;
  const uint8_t self_idx = (uint8_t)(s->agent_id - 1u);
  uint8_t max_assign_count = (uint8_t)(AGENT_COUNT * s->bundle_limit);
  uint8_t a = 0u;
  uint8_t t = 0u;
  uint8_t pass = 0u;

  if (max_assign_count > s->task_count) {
    max_assign_count = s->task_count;
  }

  for (a = 0u; a < AGENT_COUNT; a++) {
    uint8_t i = 0u;

    agent_available[a] = bidRowFresh(s, a, now_ms);
#if DEMO_AGENT_TASK_CAP_ENABLE
    agent_task_limit[a] = demoRemainingTaskCapForAgent(s, a, self_idx);
#else
    agent_task_limit[a] = s->bundle_limit;
#endif
    agent_task_count[a] = 0u;

    for (i = 0u; i < TASK_MAX; i++) {
      agent_task_order[a][i] = 255u;
    }
  }

  for (t = 0u; t < TASK_MAX; t++) {
    task_owner[t] = 0u;
    task_bid[t] = BIDVEC_INVALID_Q;
#if ACTIVE_EXEC_RESERVATION_LOCK_ENABLE
    reservation_owner[t] = 255u;
    reservation_bid[t] = BIDVEC_INVALID_Q;
#endif

    if (t < s->task_count) {
      if (s->done[t] != 0u) {
        s->winner[t] = 0u;
        s->bid_q[t] = 0;
      } else {
        s->winner[t] = 0u;
        s->bid_q[t] = 0;
        s->winner_rx_ms[t] = 0u;
      }
    }
  }

#if ACTIVE_EXEC_RESERVATION_LOCK_ENABLE
  for (a = 0u; a < AGENT_COUNT; a++) {
    const uint8_t exec_t = execTaskForAgent(s, a, self_idx);

    if ((agent_available[a] == 0u) || (agent_task_count[a] >= agent_task_limit[a]) ||
        (exec_t >= s->task_count) || (s->done[exec_t] != 0u) || (taskIsReleased(s, exec_t) == 0u)) {
      continue;
    }

    {
      const int16_t bid = bidForAgentTask(s, a, exec_t);
      const uint8_t have_reservation = (reservation_owner[exec_t] < AGENT_COUNT) ? 1u : 0u;

      if (bid == BIDVEC_INVALID_Q) {
        continue;
      }

      if (candidateBetter(bid, a, exec_t, have_reservation, reservation_bid[exec_t],
                          reservation_owner[exec_t], exec_t) != 0u) {
        reservation_owner[exec_t] = a;
        reservation_bid[exec_t] = bid;
      }
    }
  }

  for (t = 0u; t < s->task_count; t++) {
    const uint8_t owner_idx = reservation_owner[t];

    if (owner_idx >= AGENT_COUNT) {
      continue;
    }

    if (agent_task_count[owner_idx] >= agent_task_limit[owner_idx]) {
      continue;
    }

    task_owner[t] = (uint8_t)(owner_idx + 1u);
    task_bid[t] = reservation_bid[t];
    agent_task_order[owner_idx][agent_task_count[owner_idx]] = t;
    agent_task_count[owner_idx]++;
  }
#endif

  for (pass = 0u; pass < max_assign_count; pass++) {
    uint8_t have_best = 0u;
    uint8_t best_agent = 0u;
    uint8_t best_task = 0u;
    int16_t best_bid = BIDVEC_INVALID_Q;

    for (a = 0u; a < AGENT_COUNT; a++) {
      if ((agent_available[a] == 0u) || (agent_task_count[a] >= agent_task_limit[a])) {
        continue;
      }

      for (t = 0u; t < s->task_count; t++) {
        const int16_t bid = bidForAgentTask(s, a, t);

        if ((task_owner[t] != 0u) || (bid == BIDVEC_INVALID_Q)) {
          continue;
        }

        if (candidateBetter(bid, a, t, have_best, best_bid, best_agent, best_task) != 0u) {
          have_best = 1u;
          best_agent = a;
          best_task = t;
          best_bid = bid;
        }
      }
    }

    if (have_best == 0u) {
      break;
    }

    task_owner[best_task] = (uint8_t)(best_agent + 1u);
    task_bid[best_task] = best_bid;
    agent_task_order[best_agent][agent_task_count[best_agent]] = best_task;
    agent_task_count[best_agent]++;
  }

  s->bundle_len = 0u;
  s->path_len = 0u;

  for (t = 0u; t < s->task_count; t++) {
    if ((s->done[t] != 0u) || (task_owner[t] == 0u)) {
      continue;
    }

    s->winner[t] = task_owner[t];
    s->bid_q[t] = task_bid[t];

    if (task_owner[t] != s->agent_id) {
      const uint8_t owner_idx = (uint8_t)(task_owner[t] - 1u);
      s->winner_rx_ms[t] = s->peer_bid_rx_ms[owner_idx];
    }
  }

  for (pass = 0u; pass < agent_task_count[self_idx]; pass++) {
    const uint8_t local_task = agent_task_order[self_idx][pass];

    if ((local_task < s->task_count) && (s->done[local_task] == 0u) &&
        (s->winner[local_task] == s->agent_id)) {
      s->bundle[s->bundle_len] = local_task;
      s->path[s->path_len] = local_task;
      s->bundle_len++;
      s->path_len++;
    }
  }

  updateLocalFp(s);

  if (old_exec != s->exec_task) {
    setAssignmentBroadcastBurst(s);

    if ((old_exec < s->task_count) && (s->done[old_exec] == 0u)) {
      s->auction_switch_count++;

      if (now_ms != 0u) {
        s->replan_hold_until_ms = now_ms + (uint32_t)REPLAN_HOLD_MS;
      }
    }

  }
}

static void initTasks(CbbaTask* tasks) {
  uint8_t i = 0u;

  for (i = 0u; i < TASK_MAX; i++) {
    tasks[i].active = false;
    tasks[i].value_q = 0;
    tasks[i].pos.x_m = 0.0f;
    tasks[i].pos.y_m = 0.0f;
    tasks[i].exit_pos.x_m = 0.0f;
    tasks[i].exit_pos.y_m = 0.0f;
  }

  tasks[0].active = true;
  tasks[0].value_q = TASK0_VALUE_Q;
  tasks[0].pos.x_m = TASK0_X_M;
  tasks[0].pos.y_m = TASK0_Y_M;
  tasks[0].exit_pos.x_m = TASK0_END_X_M;
  tasks[0].exit_pos.y_m = TASK0_END_Y_M;

  tasks[1].active = true;
  tasks[1].value_q = TASK1_VALUE_Q;
  tasks[1].pos.x_m = TASK1_X_M;
  tasks[1].pos.y_m = TASK1_Y_M;
  tasks[1].exit_pos.x_m = TASK1_END_X_M;
  tasks[1].exit_pos.y_m = TASK1_END_Y_M;

  tasks[2].active = true;
  tasks[2].value_q = TASK2_VALUE_Q;
  tasks[2].pos.x_m = TASK2_X_M;
  tasks[2].pos.y_m = TASK2_Y_M;
  tasks[2].exit_pos.x_m = TASK2_END_X_M;
  tasks[2].exit_pos.y_m = TASK2_END_Y_M;

}

void Cbba_Init(CbbaState* s, uint8_t agent_id, CbbaVec2 start_pos) {
  uint8_t i = 0u;

  memset(s, 0, sizeof(*s));

  s->agent_id = agent_id;
  s->task_count = TASK_COUNT_RUNTIME;
  s->bundle_limit = BUNDLE_LIMIT;
  s->self_pos = start_pos;
  s->exec_task = 255u;
  s->mission_done_since_ms = 0u;
  s->done_mask = 0u;
  s->local_done_count = 0u;
  s->claim_rr = 0u;
  s->done_rr = 0u;
  s->claim_rx_count = 0u;
  s->claim_loss_count = 0u;
  s->late_release_count = 0u;
  s->bidvec_rx_count = 0u;
  s->auction_switch_count = 0u;
  s->last_bidvec_tx_ms = 0u;
  s->bidvec_seq = 0u;
  s->last_bid_refresh_ms = 0u;
  s->replan_hold_until_ms = 0u;
  s->claim_burst_left = CLAIM_BURST_COUNT;

  initTasks(s->tasks);

  for (i = 0u; i < TASK_MAX; i++) {
    uint8_t a = 0u;

    s->winner[i] = 0u;
    s->bid_q[i] = 0;
    s->ver[i] = 0u;
    s->done[i] = 0u;
    s->done_enter_ms[i] = 0u;
    s->winner_rx_ms[i] = 0u;
    s->lane_entered[i] = 0u;

    for (a = 0u; a < AGENT_COUNT; a++) {
      s->peer_bid_q[a][i] = BIDVEC_INVALID_Q;
    }
  }

  for (i = 0u; i < AGENT_COUNT; i++) {
    s->peer_done_mask[i] = 0u;
    s->peer_bid_rx_ms[i] = 0u;
    s->peer_bid_valid[i] = 0u;
    s->peer_exec_task[i] = 255u;
  }

  addBundleTasks(s);
  updateLocalFp(s);
}

void Cbba_SetPose(CbbaState* s, CbbaVec2 pos) {
  s->self_pos = pos;
}

void Cbba_HandleClaim(CbbaState* s, const msg_claim_t* m, uint32_t now_ms) {
  const uint8_t t = m->task_id;
  const uint8_t sender_agent = appAgentIdFromRadioLow(m->src_id);
  const uint8_t old_winner = (t < TASK_MAX) ? s->winner[t] : 0u;
  const uint8_t lost_bundle_idx = (t < TASK_MAX) ? bundleFindIndex(s, t) : 255u;
  uint8_t peer_wins = 0u;
  uint8_t i = 0u;
  uint8_t table_changed = 0u;

  if (t >= s->task_count) {
    return;
  }

  if (m->done_mask != 0u) {
    for (i = 0u; i < s->task_count; i++) {
      if (((m->done_mask & ((uint16_t)1u << i)) != 0u) && (s->done[i] == 0u)) {
        if (s->ver[i] == 0u) {
          s->ver[i] = 1u;
        }
        markTaskDone(s, i, 0u);
        table_changed = 1u;
      }
    }
  }

  if (s->done[t] != 0u) {
    if (table_changed != 0u) {
      pruneInvalidAssignments(s);
      updateLocalFp(s);
    }
    return;
  }

  if (sender_agent == s->agent_id) {
    if (table_changed != 0u) {
      pruneInvalidAssignments(s);
      updateLocalFp(s);
    }
    return;
  }

  s->claim_rx_count++;

#if ROLLING_AUCTION_ENABLE
  {
    const uint8_t sender_idx = appNodeIndexFromId(m->src_id);

    if (sender_idx < AGENT_COUNT) {
      s->peer_bid_q[sender_idx][t] = m->bid_q;
      s->peer_exec_task[sender_idx] = t;
      s->peer_done_mask[sender_idx] = (uint16_t)(s->peer_done_mask[sender_idx] | m->done_mask);
      s->peer_bid_rx_ms[sender_idx] = now_ms;
      s->peer_bid_valid[sender_idx] = 1u;
    }
  }
#endif

  clearPeerReservationsOutsideMask(s, sender_agent,
                                   (uint16_t)(m->heard_mask | ((uint16_t)1u << t)));

  peer_wins = peerReservationWins(s, t, sender_agent, m->bid_q);

  if (peer_wins == 0u) {
    if ((old_winner == s->agent_id) && (lost_bundle_idx != 255u)) {
      setClaimBurst(s);
    }
    if (table_changed != 0u) {
      pruneInvalidAssignments(s);
      updateLocalFp(s);
    }
    return;
  }

#if CLAIM_IMMEDIATE_SUFFIX_RELEASE
  if ((old_winner == s->agent_id) && (lost_bundle_idx != 255u)) {
    s->claim_loss_count++;

    if (s->exec_task == t) {
      s->late_release_count++;

      if (now_ms != 0u) {
        s->replan_hold_until_ms = now_ms + (uint32_t)REPLAN_HOLD_MS;
      }
    }

    releaseSuffix(s, lost_bundle_idx);
    setClaimBurst(s);
  }
#endif

  s->winner[t] = sender_agent;
  s->bid_q[t] = m->bid_q;
  s->winner_rx_ms[t] = now_ms;

  if (m->ver > s->ver[t]) {
    s->ver[t] = m->ver;
  }

  pruneInvalidAssignments(s);

#if ROLLING_AUCTION_ENABLE
  rollingAuctionAssign(s, now_ms);
#else
  if (s->done_count < s->task_count) {
    addBundleTasks(s);
  }

  setClaimBurst(s);

  pruneInvalidAssignments(s);
  updateLocalFp(s);
#endif
}

void Cbba_HandleDone(CbbaState* s, const msg_done_t* m, uint32_t now_ms) {
  const uint8_t t = m->task_id;
  uint8_t i = 0u;
  uint16_t done_mask = 0u;
  uint8_t src_idx = 0u;

  if (t >= s->task_count) {
    return;
  }

  /* DONE is terminal mission information. It must dominate CLAIM state;
   * otherwise an old local claim/version can make a completed task look
   * available again.
   */
  if (m->ver > s->ver[t]) {
    s->ver[t] = m->ver;
  }

  if (s->ver[t] == 0u) {
    s->ver[t] = 1u;
  }

  done_mask = (uint16_t)(m->done_mask | ((uint16_t)1u << t));

  if (appIsValidNodeId(m->src_id)) {
    src_idx = appNodeIndexFromId(m->src_id);
    if (src_idx < AGENT_COUNT) {
      s->peer_done_mask[src_idx] = (uint16_t)(s->peer_done_mask[src_idx] | done_mask);
      if ((s->peer_exec_task[src_idx] < TASK_MAX) &&
          ((done_mask & ((uint16_t)1u << s->peer_exec_task[src_idx])) != 0u)) {
        s->peer_exec_task[src_idx] = 255u;
      }
    }
  }

  for (i = 0u; i < s->task_count; i++) {
    if ((done_mask & ((uint16_t)1u << i)) == 0u) {
      continue;
    }

    if (s->ver[i] == 0u) {
      s->ver[i] = 1u;
    }

    markTaskDone(s, i, 0u);
  }

  pruneInvalidAssignments(s);

#if ROLLING_AUCTION_ENABLE
  rollingAuctionAssign(s, now_ms);
#else
  if (s->done_count < s->task_count) {
    addBundleTasks(s);
  }

  setClaimBurst(s);

  pruneInvalidAssignments(s);
  updateLocalFp(s);
#endif
}

void Cbba_HandleBidVec(CbbaState* s, const msg_bidvec_t* m, uint32_t now_ms) {
  uint8_t i = 0u;
  uint8_t src_idx = 0u;

  if ((s == (CbbaState*)0) || (m == (const msg_bidvec_t*)0)) {
    return;
  }

  absorbDoneMask(s, m->done_mask);

  if ((!appIsValidNodeId(m->src_id)) || (appAgentIdFromRadioLow(m->src_id) == s->agent_id)) {
    return;
  }

  src_idx = appNodeIndexFromId(m->src_id);

  for (i = 0u; i < TASK_MAX; i++) {
    if (i < m->task_count) {
      s->peer_bid_q[src_idx][i] = m->bid_q[i];
    } else {
      s->peer_bid_q[src_idx][i] = BIDVEC_INVALID_Q;
    }
  }

  s->peer_done_mask[src_idx] = (uint16_t)(s->peer_done_mask[src_idx] | m->done_mask);
  s->peer_bid_rx_ms[src_idx] = now_ms;
  s->peer_bid_valid[src_idx] = 1u;

  if ((m->exec_task < m->task_count) && ((m->done_mask & ((uint16_t)1u << m->exec_task)) == 0u) &&
      (s->done[m->exec_task] == 0u)) {
    s->peer_exec_task[src_idx] = m->exec_task;
  } else {
    s->peer_exec_task[src_idx] = 255u;
  }
  s->bidvec_rx_count++;

  rollingAuctionAssign(s, now_ms);
}

void Cbba_LocalStep(CbbaState* s, uint32_t now_ms) {
#if ROLLING_AUCTION_ENABLE
  uint8_t t = 0u;

  rollingAuctionAssign(s, now_ms);

  for (t = 0u; t < s->task_count; t++) {
    if ((s->done[t] != 0u) || (s->winner[t] != s->agent_id)) {
      s->lane_entered[t] = 0u;
    }
  }
#else
  uint8_t i = 0u;

  pruneStalePeerReservations(s, now_ms);
  pruneInvalidAssignments(s);
  refreshLocalClaimBids(s, now_ms);

  for (i = 0u; i < s->bundle_len; i++) {
    const uint8_t t = s->bundle[i];

    if ((t < s->task_count) && (s->winner[t] != s->agent_id)) {
      releaseSuffix(s, i);
      break;
    }
  }

  pruneInvalidAssignments(s);

  if ((s->path_len == 0u || s->bundle_len < s->bundle_limit) && (s->done_count < s->task_count)) {
    addBundleTasks(s);
  }

  pruneInvalidAssignments(s);
  updateLocalFp(s);
#endif
}

void Cbba_MarkReachedDone(CbbaState* s, uint32_t now_ms) {
  uint8_t t = 0u;
  uint8_t done_changed = 0u;

  for (t = 0u; t < s->task_count; t++) {
    const uint8_t is_point_task =
        (dist_m(s->tasks[t].pos, s->tasks[t].exit_pos) <= 0.001f) ? 1u : 0u;

    if ((s->done[t] != 0u) || (taskIsReleased(s, t) == 0u)) {
      s->done_enter_ms[t] = 0u;
      s->lane_entered[t] = 0u;
      continue;
    }

    if (is_point_task != 0u) {
      if (dist_m(s->self_pos, s->tasks[t].pos) <= DONE_RADIUS_M) {
        if (s->done_enter_ms[t] == 0u) {
          s->done_enter_ms[t] = now_ms;
        }

        if ((now_ms - s->done_enter_ms[t]) >= DONE_DWELL_MS) {
          const uint8_t was_done = s->done[t];
          markTaskDone(s, t, 1u);

          if ((was_done == 0u) && (s->local_done_count < TASK_MAX)) {
            s->local_done_count++;
          }

          updateLocalFp(s);
          done_changed = 1u;
        }
      } else {
        s->done_enter_ms[t] = 0u;
      }

      s->lane_entered[t] = 0u;
      continue;
    }

    if (s->lane_entered[t] == 0u) {
      if (dist_m(s->self_pos, s->tasks[t].pos) <= LANE_ENTRY_RADIUS_M) {
        s->lane_entered[t] = 1u;
        s->done_enter_ms[t] = 0u;
      } else {
        s->done_enter_ms[t] = 0u;
      }

      continue;
    }

    if (dist_m(s->self_pos, s->tasks[t].exit_pos) <= DONE_RADIUS_M) {
      if (s->done_enter_ms[t] == 0u) {
        s->done_enter_ms[t] = now_ms;
      }

      if ((now_ms - s->done_enter_ms[t]) >= DONE_DWELL_MS) {
        const uint8_t was_done = s->done[t];
        markTaskDone(s, t, 1u);

        if ((was_done == 0u) && (s->local_done_count < TASK_MAX)) {
          s->local_done_count++;
        }

        updateLocalFp(s);
        done_changed = 1u;
      }
    } else {
      s->done_enter_ms[t] = 0u;
    }
  }

  if (done_changed != 0u) {
    for (t = 0u; t < s->task_count; t++) {
      s->done_enter_ms[t] = 0u;
    }

    updateLocalFp(s);

#if ROLLING_AUCTION_ENABLE
    rollingAuctionAssign(s, now_ms);
    setAssignmentBroadcastBurst(s);
#else
    if (s->done_count < s->task_count) {
      addBundleTasks(s);
      setAssignmentBroadcastBurst(s);
      updateLocalFp(s);
    }
#endif

    if ((s->done_count >= s->task_count) && (s->mission_done_since_ms == 0u)) {
      s->mission_done_since_ms = now_ms;
    }
  }
}

bool Cbba_GetExecTarget(const CbbaState* s, CbbaVec2* out_target, uint8_t* out_phase) {
  uint8_t t = 0u;

  if ((s == (const CbbaState*)0) || (out_target == (CbbaVec2*)0)) {
    return false;
  }

  t = s->exec_task;

  if ((t >= s->task_count) || (s->done[t] != 0u) || (taskIsReleased(s, t) == 0u) ||
      (s->winner[t] != s->agent_id)) {
    return false;
  }

  if (s->lane_entered[t] != 0u) {
    *out_target = s->tasks[t].exit_pos;
    if (out_phase != (uint8_t*)0) {
      *out_phase = 1u;
    }
  } else {
    *out_target = s->tasks[t].pos;
    if (out_phase != (uint8_t*)0) {
      *out_phase = 0u;
    }
  }

  return true;
}

bool Cbba_MakeClaimMsg(CbbaState* s, uint32_t now_ms, msg_claim_t* out) {
  uint8_t t = 255u;
  uint8_t rr_used = 0u;

  if ((out == (msg_claim_t*)0) || (s->bundle_len == 0u)) {
    return false;
  }

  if ((s->claim_burst_left == 0u) && (s->last_claim_tx_ms != 0u) &&
      ((now_ms - s->last_claim_tx_ms) < CLAIM_TX_PERIOD_MS)) {
    return false;
  }

  rr_used = s->claim_rr;

  if (rr_used >= s->bundle_len) {
    rr_used = 0u;
  }

  t = s->bundle[rr_used];

  s->claim_rr++;

  if (s->claim_rr >= s->bundle_len) {
    s->claim_rr = 0u;
  }

  if ((t >= s->task_count) || (s->done[t] != 0u) || (s->winner[t] != s->agent_id)) {
    pruneInvalidAssignments(s);
    updateLocalFp(s);
    return false;
  }

  memset(out, 0, sizeof(*out));

  out->type = MSG_CLAIM;
  out->src_id = appNodeIdFromIndex((uint8_t)(s->agent_id - 1u));
  out->tx_id = out->src_id;
  out->seq = ++s->tx_seq;
  out->ttl = TTL_MAX;
  out->hop = 0u;
  out->tx_x_cm = cmFromMeter(s->self_pos.x_m);
  out->tx_y_cm = cmFromMeter(s->self_pos.y_m);
  out->task_id = t;
  out->bid_q = s->bid_q[t];
  out->ver = s->ver[t];
  out->path_idx = rr_used;
  out->heard_mask = reservationMask(s);
  out->done_mask = s->done_mask;

  s->last_claim_tx_ms = now_ms;

  if (s->claim_burst_left > 0u) {
    s->claim_burst_left--;
  }

  return true;
}

bool Cbba_MakeDoneMsg(CbbaState* s, uint32_t now_ms, msg_done_t* out) {
  uint8_t k = 0u;

  if (out == (msg_done_t*)0) {
    return false;
  }

  if ((s->last_done_tx_ms != 0u) && ((now_ms - s->last_done_tx_ms) < DONE_REPEAT_PERIOD_MS)) {
    return false;
  }

  for (k = 0u; k < s->task_count; k++) {
    const uint8_t t = (uint8_t)((s->done_rr + k) % s->task_count);

    if ((s->done[t] != 0u) && (s->ver[t] != 0u)) {
      memset(out, 0, sizeof(*out));

      out->type = MSG_DONE;
      out->src_id = appNodeIdFromIndex((uint8_t)(s->agent_id - 1u));
      out->tx_id = out->src_id;
      out->seq = ++s->tx_seq;
      out->ttl = TTL_MAX;
      out->hop = 0u;
      out->tx_x_cm = cmFromMeter(s->self_pos.x_m);
      out->tx_y_cm = cmFromMeter(s->self_pos.y_m);
      out->task_id = t;
      out->ver = s->ver[t];
      out->done_mask = s->done_mask;

      s->done_rr = (uint8_t)((t + 1u) % s->task_count);
      s->last_done_tx_ms = now_ms;
      return true;
    }
  }

  return false;
}

bool Cbba_MakeBidVecMsg(CbbaState* s, uint32_t now_ms, msg_bidvec_t* out) {
  uint8_t t = 0u;
#if DEMO_AGENT_TASK_CAP_ENABLE
  uint8_t self_cap_reached = 0u;
#endif

  if ((s == (CbbaState*)0) || (out == (msg_bidvec_t*)0)) {
    return false;
  }

  if ((s->last_bidvec_tx_ms != 0u) && ((now_ms - s->last_bidvec_tx_ms) < BIDVEC_TX_PERIOD_MS)) {
    return false;
  }

#if DEMO_AGENT_TASK_CAP_ENABLE
  self_cap_reached = demoSelfTaskCapReached(s);
#endif

  memset(out, 0, sizeof(*out));

  out->type = MSG_BIDVEC;
  out->src_id = appNodeIdFromIndex((uint8_t)(s->agent_id - 1u));
  out->tx_id = out->src_id;
  out->seq = ++s->bidvec_seq;
  out->ttl = TTL_MAX;
  out->hop = 0u;
  out->tx_x_cm = cmFromMeter(s->self_pos.x_m);
  out->tx_y_cm = cmFromMeter(s->self_pos.y_m);
  out->done_mask = s->done_mask;
  out->task_count = s->task_count;
#if DEMO_AGENT_TASK_CAP_ENABLE
  out->exec_task = (self_cap_reached != 0u) ? 255u : s->exec_task;
#else
  out->exec_task = s->exec_task;
#endif

  for (t = 0u; t < TASK_MAX; t++) {
#if DEMO_AGENT_TASK_CAP_ENABLE
    if (self_cap_reached != 0u) {
      out->bid_q[t] = BIDVEC_INVALID_Q;
      continue;
    }
#endif

    if ((t < s->task_count) && (taskIsReleased(s, t) != 0u) && (s->done[t] == 0u)) {
      out->bid_q[t] = qbid_from_task(&s->tasks[t], s->self_pos);
    } else {
      out->bid_q[t] = BIDVEC_INVALID_Q;
    }
  }

  s->last_bidvec_tx_ms = now_ms;
  return true;
}

void Cbba_InitPeerCache(PeerMissionCache* c) {
  memset(c, 0, sizeof(*c));

  c->src_id = 0u;
  c->exec_task = 255u;
}

void Cbba_UpdatePeerCacheFromClaim(PeerMissionCache* c, const msg_claim_t* m) {
  const uint8_t t = m->task_id;
  uint8_t i = 0u;
  const uint8_t peer_agent = appAgentIdFromRadioLow(m->src_id);

  if ((c == (PeerMissionCache*)0) || (m == (const msg_claim_t*)0)) {
    return;
  }

  c->valid = 1u;
  c->src_id = m->src_id;
  c->task_count_total = TASK_COUNT_RUNTIME;
  c->exec_task = t;

  if (t >= TASK_MAX) {
    return;
  }

  for (i = 0u; i < TASK_COUNT_RUNTIME; i++) {
    if ((c->done[i] == 0u) && (c->winner[i] == peer_agent) &&
        (((uint16_t)(m->heard_mask | ((uint16_t)1u << t)) & ((uint16_t)1u << i)) == 0u)) {
      c->winner[i] = 0u;
      c->bid_q[i] = 0;
    }
  }

  if (c->done[t] == 0u) {
    c->winner[t] = peer_agent;
    c->bid_q[t] = m->bid_q;
    c->ver[t] = m->ver;
  }
}

void Cbba_UpdatePeerCacheFromDone(PeerMissionCache* c, const msg_done_t* m) {
  const uint8_t t = m->task_id;
  uint8_t i = 0u;
  uint16_t done_mask = 0u;

  if ((c == (PeerMissionCache*)0) || (m == (const msg_done_t*)0)) {
    return;
  }

  c->valid = 1u;
  c->src_id = m->src_id;
  c->task_count_total = TASK_COUNT_RUNTIME;

  if (t >= TASK_MAX) {
    return;
  }

  done_mask = (uint16_t)(m->done_mask | ((uint16_t)1u << t));

  for (i = 0u; i < TASK_COUNT_RUNTIME; i++) {
    if ((done_mask & ((uint16_t)1u << i)) == 0u) {
      continue;
    }

    c->done[i] = 1u;
    c->done_mask = (uint16_t)(c->done_mask | ((uint16_t)1u << i));
    c->winner[i] = 0u;
    c->bid_q[i] = 0;

    if ((i == t) && (m->ver > c->ver[i])) {
      c->ver[i] = m->ver;
    } else if (c->ver[i] == 0u) {
      c->ver[i] = 1u;
    }

    if (c->exec_task == i) {
      c->exec_task = 255u;
    }
  }
}

void Cbba_UpdatePeerCacheFromBidVec(PeerMissionCache* c, const msg_bidvec_t* m) {
  uint8_t i = 0u;
  const uint8_t peer_agent = appAgentIdFromRadioLow(m->src_id);

  if ((c == (PeerMissionCache*)0) || (m == (const msg_bidvec_t*)0)) {
    return;
  }

  c->valid = 1u;
  c->src_id = m->src_id;
  c->task_count_total = m->task_count;
  c->exec_task = 255u;
  c->done_mask = (uint16_t)(c->done_mask | m->done_mask);

  for (i = 0u; i < TASK_COUNT_RUNTIME; i++) {
    if ((m->done_mask & ((uint16_t)1u << i)) != 0u) {
      c->done[i] = 1u;
      c->winner[i] = 0u;
      c->bid_q[i] = 0;
      if (c->ver[i] == 0u) {
        c->ver[i] = 1u;
      }
    } else if (c->done[i] == 0u) {
      c->winner[i] = 0u;
      c->bid_q[i] = (i < m->task_count) ? m->bid_q[i] : BIDVEC_INVALID_Q;
    }
  }

  if ((m->exec_task < TASK_COUNT_RUNTIME) && (c->done[m->exec_task] == 0u)) {
    c->exec_task = m->exec_task;
    c->winner[m->exec_task] = peer_agent;
    c->bid_q[m->exec_task] = m->bid_q[m->exec_task];
  }
}

void Cbba_AbsorbGlobalDoneMask(CbbaState* self, const PeerMissionCache* peer1,
                               const PeerMissionCache* peer2, uint32_t now_ms) {
  const uint16_t mask = Cbba_GetGlobalDoneMask(self, peer1, peer2);
  const uint8_t changed = absorbDoneMask(self, mask);

  if (changed != 0u) {
    pruneInvalidAssignments(self);
    updateLocalFp(self);

#if ROLLING_AUCTION_ENABLE
    rollingAuctionAssign(self, now_ms);
#else
    if (self->done_count < self->task_count) {
      addBundleTasks(self);
    }

    setClaimBurst(self);
    pruneInvalidAssignments(self);
    updateLocalFp(self);
#endif
  }
}

uint16_t Cbba_GetGlobalDoneMask(const CbbaState* self, const PeerMissionCache* peer1,
                                const PeerMissionCache* peer2) {
  uint16_t mask = 0u;
  const uint16_t active_mask = taskMaskForCount(self->task_count);

  mask = (uint16_t)(self->done_mask & active_mask);

  if (peer1->valid != 0u) {
    mask = (uint16_t)(mask | (peer1->done_mask & active_mask));
  }

  if (peer2->valid != 0u) {
    mask = (uint16_t)(mask | (peer2->done_mask & active_mask));
  }

  return mask;
}

uint8_t Cbba_GetGlobalDoneCount(const CbbaState* self, const PeerMissionCache* peer1,
                                const PeerMissionCache* peer2) {
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
