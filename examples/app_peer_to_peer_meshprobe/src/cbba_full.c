#include "cbba_full.h"

#include <math.h>
#include <string.h>

#include "ids.h"

#define DEBUG_MODULE "CBBA"
#include "debug.h"

/* ── 내부 상수 ───────────────────────────────────────────────────────────────*/
typedef enum { ACT_LEAVE = 0, ACT_UPDATE, ACT_RESET } CbbaAction;

/* ── 거리 / 입찰가 계산 ───────────────────────────────────────────────────── */

static float dist_m(CbbaVec2 a, CbbaVec2 b) {
  const float dx = a.x_m - b.x_m;
  const float dy = a.y_m - b.y_m;
  return sqrtf((dx * dx) + (dy * dy));
}

/* 경로 추가 비용 delta_m → 고정소수 입찰가 (작을수록 좋아 부호 반전) */
static int16_t qbid_from_delta(float delta_m) {
  int32_t q = (int32_t)lrintf(10000.0f - (delta_m * 1000.0f));

  if (q > 32767) {
    q = 32767;
  }

  if (q < -32768) {
    q = -32768;
  }

  return (int16_t)q;
}

/* ── 경로 / 번들 유틸리티 ────────────────────────────────────────────────── */

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

static void pathRemove(CbbaState* s, uint8_t task_id) {
  const uint8_t idx = pathFindIndex(s, task_id);
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
  const uint8_t idx = bundleFindIndex(s, task_id);
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

/* ── 경로 삽입 비용 계산 ─────────────────────────────────────────────────── */

static float insertionDeltaCost(const CbbaState* s, uint8_t task_id, uint8_t ins_idx) {
  CbbaVec2 prev_p;
  CbbaVec2 next_p;
  const CbbaVec2 task_p = s->tasks[task_id].pos;

  if (ins_idx == 0u) {
    prev_p = s->self_pos;
  } else {
    prev_p = s->tasks[s->path[ins_idx - 1u]].pos;
  }

  if (ins_idx >= s->path_len) {
    return dist_m(prev_p, task_p);
  }

  next_p = s->tasks[s->path[ins_idx]].pos;

  return dist_m(prev_p, task_p) + dist_m(task_p, next_p) - dist_m(prev_p, next_p);
}

/* task_id를 path의 모든 위치에 삽입해보고 최소 비용 반환 */
static int16_t bestInsertionBid(const CbbaState* s, uint8_t task_id, uint8_t* best_idx) {
  float best_delta = 1.0e9f;
  uint8_t i = 0u;
  uint8_t idx_best = 0u;

  for (i = 0u; i <= s->path_len; i++) {
    const float d = insertionDeltaCost(s, task_id, i);

    if (d < best_delta) {
      best_delta = d;
      idx_best = i;
    }
  }

  *best_idx = idx_best;
  return qbid_from_delta(best_delta);
}

/* ── 내부 상태 갱신 ──────────────────────────────────────────────────────── */

static void updateLocalState(CbbaState* s) {
  uint8_t i = 0u;
  uint8_t done_cnt = 0u;

  for (i = 0u; i < s->task_count; i++) {
    if (s->done[i] != 0u) {
      done_cnt++;
    }
  }

  s->done_count = done_cnt;
  s->exec_task = (s->path_len > 0u) ? s->path[0] : 255u;
}

/* ── 번들 suffix 해제 (start_idx 이후 항목 전부 제거 + winner/bid 초기화) ── */
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
      }

      pathRemove(s, t);
    }
  }

  s->bundle_len = start_idx;
}

/* ── Phase 1: 번들에 task 추가 (한계 점수 기반 greedy) ───────────────────── */
static void addBundleTasks(CbbaState* s) {
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

      my_bid = bestInsertionBid(s, t, &ins_idx);

      /* h_ij = (my_bid > y_ij) OR (tiebreak by agent_id) */
      if ((s->winner[t] == 0u) || (my_bid > s->bid_q[t]) ||
          ((my_bid == s->bid_q[t]) && (s->agent_id < s->winner[t]))) {
        if (my_bid > best_bid) {
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
  }
}

/* ── Table 1 충돌 해결 (Choi et al. 2009) ───────────────────────────────── */

/* seqA > seqB? uint8_t wraparound 안전 비교 */
static bool seqNewer(uint8_t a, uint8_t b) {
  return (int8_t)(a - b) > 0;
}

/* k의 agent_seq[agent_id] > 내 last_seq[agent_id]? (k가 더 최신 정보 보유) */
static bool senderKnowsMoreAbout(const msg_cbba_state_t* msg, const CbbaState* s,
                                 uint8_t agent_id) {
  if (agent_id == 0u || agent_id > AGENT_COUNT) {
    return false;
  }

  const uint8_t idx = (uint8_t)(agent_id - 1u);
  return seqNewer(msg->agent_seq[idx], s->last_seq[idx]);
}

/* 내 last_seq[agent_id] > k의 agent_seq[agent_id]? (내가 더 최신 정보 보유) */
static bool iKnowMoreAbout(const CbbaState* s, const msg_cbba_state_t* msg, uint8_t agent_id) {
  if (agent_id == 0u || agent_id > AGENT_COUNT) {
    return false;
  }

  const uint8_t idx = (uint8_t)(agent_id - 1u);
  return seqNewer(s->last_seq[idx], msg->agent_seq[idx]);
}

/*
 * Table 1 전체 케이스 구현
 *   msg : sender k 의 CBBA 상태 패킷
 *   s   : receiver i (나) 의 CBBA 상태
 *   k   : sender  의 agent_id (1-3)
 *   i   : receiver의 agent_id (1-3, == s->agent_id)
 *   y_kj, y_ij : 해당 task의 입찰가
 *   z_kj, z_ij : 해당 task의 winner (agent_id, 0=없음)
 */
static CbbaAction resolveTable1(const msg_cbba_state_t* msg, const CbbaState* s, uint8_t k,
                                uint8_t i, int16_t y_kj, int16_t y_ij, uint8_t z_kj, uint8_t z_ij) {
  /* ── Case A: z_kj = k (sender가 자기가 winner) ─────────────────────────── */
  if (z_kj == k) {
    if (z_ij == i) {
      /* sender가 나를 outbid 했으면 update */
      return (y_kj > y_ij) ? ACT_UPDATE : ACT_LEAVE;
    }

    if (z_ij == k) {
      /* 이미 k가 winner 로 동의 → update (최신 bid 반영) */
      return ACT_UPDATE;
    }

    if (z_ij == 0u) {
      return ACT_UPDATE;
    }

    /* z_ij = m (제3자): k가 m 보다 최신 정보를 갖거나 bid가 높으면 update */
    return (senderKnowsMoreAbout(msg, s, z_ij) || (y_kj > y_ij)) ? ACT_UPDATE : ACT_LEAVE;
  }

  /* ── Case B: z_kj = i (sender가 내가 winner라 생각) ────────────────────── */
  if (z_kj == i) {
    if (z_ij == i) {
      return ACT_LEAVE;
    }

    if (z_ij == k) {
      return ACT_RESET;
    }

    if (z_ij == 0u) {
      return ACT_LEAVE;
    }

    /* z_ij = m (제3자): k가 m보다 최신 정보면 reset (k가 나를 winner로 보는데 내가 m을 winner로
     * 알고 있어 불일치) */
    return senderKnowsMoreAbout(msg, s, z_ij) ? ACT_RESET : ACT_LEAVE;
  }

  /* ── Case D: z_kj = 0 (sender가 winner 없다고 생각) ────────────────────── */
  if (z_kj == 0u) {
    if (z_ij == i) {
      return ACT_LEAVE;
    }

    if (z_ij == k) {
      /* k가 자기 winner 기록을 지웠으니 신뢰 */
      return ACT_UPDATE;
    }

    if (z_ij == 0u) {
      return ACT_LEAVE;
    }

    /* z_ij = m (제3자): k가 더 최신 정보면 update */
    return senderKnowsMoreAbout(msg, s, z_ij) ? ACT_UPDATE : ACT_LEAVE;
  }

  /* ── Case C: z_kj = m (sender가 제3자 m이 winner라 생각) ────────────────
   *  z_kj != k, z_kj != i, z_kj != 0 → 반드시 제3자 */
  {
    const uint8_t m = z_kj;
    const bool s_km_gt = senderKnowsMoreAbout(msg, s, m); /* s_k[m] > s_i[m] */

    if (z_ij == i) {
      /* 나를 winner로 알고 있는데 k가 m이 winner라 함 → 둘 다 조건 충족 시 update */
      return (s_km_gt && (y_kj > y_ij)) ? ACT_UPDATE : ACT_LEAVE;
    }

    if (z_ij == k) {
      /* k에 대한 내 기록 vs k의 m 기록 */
      return s_km_gt ? ACT_UPDATE : ACT_RESET;
    }

    if (z_ij == m) {
      /* 같은 제3자 m이 winner → k가 더 최신이면 update */
      return s_km_gt ? ACT_UPDATE : ACT_LEAVE;
    }

    if (z_ij == 0u) {
      return s_km_gt ? ACT_UPDATE : ACT_LEAVE;
    }

    /* z_ij = n (또 다른 제3자) */
    {
      const uint8_t n = z_ij;
      const bool s_kn_gt = senderKnowsMoreAbout(msg, s, n); /* s_k[n] > s_i[n] */
      const bool s_im_gt = iKnowMoreAbout(s, msg, m);       /* s_i[m] > s_k[m] */

      if (s_km_gt && s_kn_gt) {
        return ACT_UPDATE;
      }

      if (s_km_gt && (y_kj > y_ij)) {
        return ACT_UPDATE;
      }

      if (s_kn_gt && s_im_gt) {
        return ACT_RESET;
      }

      return ACT_LEAVE;
    }
  }
}

/* ── 공개 API ─────────────────────────────────────────────────────────────── */

void Cbba_Init(CbbaState* s, uint8_t agent_id, CbbaVec2 start_pos, uint8_t task_count,
               uint8_t bundle_limit, const CbbaTask* tasks) {
  uint8_t i = 0u;

  memset(s, 0, sizeof(*s));

  s->agent_id = agent_id;
  s->task_count = (task_count <= TASK_MAX) ? task_count : TASK_MAX;
  s->bundle_limit = bundle_limit;
  s->self_pos = start_pos;
  s->exec_task = 255u;

  for (i = 0u; i < s->task_count; i++) {
    s->tasks[i] = tasks[i];
  }

  /* 나머지 슬롯은 비활성 */
  for (i = s->task_count; i < TASK_MAX; i++) {
    s->tasks[i].active = false;
  }

  addBundleTasks(s);
  updateLocalState(s);
}

void Cbba_SetPose(CbbaState* s, CbbaVec2 pos) {
  s->self_pos = pos;
}

/* Phase 1: outbid 된 task suffix 해제 후 bundle 재구성 */
void Cbba_LocalStep(CbbaState* s, uint32_t now_ms) {
  uint8_t i = 0u;
  (void)now_ms;

  /* 번들 순서대로 확인: winner 가 더 이상 나 자신이 아닌 task 발견 시 suffix 해제 */
  for (i = 0u; i < s->bundle_len; i++) {
    const uint8_t t = s->bundle[i];

    if ((t < s->task_count) && (s->winner[t] != s->agent_id)) {
      releaseSuffix(s, i);
      break;
    }
  }

  if ((s->path_len == 0u || s->bundle_len < s->bundle_limit) && (s->done_count < s->task_count)) {
    addBundleTasks(s);
  }

  updateLocalState(s);
}

/*
 * Phase 2: 이웃 k의 CBBA 상태를 수신해 Table 1 충돌 해결 + s_i 갱신
 *
 * done_mask 처리 우선순서:
 *   1) 상대방 done_mask 에서 완료 표시된 task 를 내 상태에 먼저 반영
 *   2) 그 이후 bid/winner 충돌 해결 (완료 task 는 skip)
 */
void Cbba_HandleCbbaState(CbbaState* s, const msg_cbba_state_t* m) {
  uint8_t t = 0u;
  uint8_t ai = 0u;
  uint8_t k_idx = 0u;
  uint8_t k = 0u; /* sender agent_id */
  const uint8_t i = s->agent_id;

  if (m == (const msg_cbba_state_t*)0) {
    return;
  }

  k = appAgentIdFromRadioLow(m->src_id);
  k_idx = appNodeIndexFromId(m->src_id);

  if (k == i || k_idx >= AGENT_COUNT) {
    return;
  }

  /* ── 1. done_mask 전파: 상대가 완료한 task 를 내 상태에 먼저 반영 ─────── */
  for (t = 0u; t < s->task_count; t++) {
    if ((m->done_mask >> t) & 0x01u) {
      if (s->done[t] == 0u) {
        const uint8_t bi = bundleFindIndex(s, t);

        if (bi != 255u) {
          releaseSuffix(s, bi);
        }

        s->done[t] = 1u;
        s->done_mask |= (uint8_t)(1u << t);
        s->winner[t] = 0u;
        s->bid_q[t] = 0;
        pathRemove(s, t);
        bundleRemove(s, t);
      }
    }
  }

  /* ── 2. Table 1 충돌 해결 ─────────────────────────────────────────────── */
  for (t = 0u; t < s->task_count; t++) {
    const uint8_t z_kj = m->winner[t];
    const uint8_t z_ij = s->winner[t];
    const int16_t y_kj = m->bid[t];
    const int16_t y_ij = s->bid_q[t];
    CbbaAction act;

    if (s->done[t] != 0u) {
      continue; /* 완료 task 는 건너뜀 */
    }

    act = resolveTable1(m, s, k, i, y_kj, y_ij, z_kj, z_ij);

    if (act == ACT_UPDATE) {
      const uint8_t bi = bundleFindIndex(s, t);

      if (bi != 255u) {
        releaseSuffix(s, bi);
      }

      s->winner[t] = z_kj;
      s->bid_q[t] = y_kj;
    } else if (act == ACT_RESET) {
      const uint8_t bi = bundleFindIndex(s, t);

      if (bi != 255u) {
        releaseSuffix(s, bi);
      }

      s->winner[t] = 0u;
      s->bid_q[t] = 0;
    }
  }

  /* ── 3. s_i 갱신 ──────────────────────────────────────────────────────── */
  s->last_seq[k_idx] = m->seq; /* 직접 수신: k 의 최신 seq 기록 */

  /* k 가 알고 있는 다른 에이전트 정보 전파 (간접 timestamp) */
  for (ai = 0u; ai < AGENT_COUNT; ai++) {
    if (ai == k_idx) {
      continue; /* k 자신은 위에서 처리 */
    }

    if (seqNewer(m->agent_seq[ai], s->last_seq[ai])) {
      s->last_seq[ai] = m->agent_seq[ai];
    }
  }

  updateLocalState(s);
}

bool Cbba_MakeCbbaStateMsg(CbbaState* s, uint32_t now_ms, msg_cbba_state_t* out) {
  uint8_t t = 0u;
  uint8_t ai = 0u;

  if (out == (msg_cbba_state_t*)0) {
    return false;
  }

  if ((s->last_cbba_tx_ms != 0u) && ((now_ms - s->last_cbba_tx_ms) < CBBA_TX_PERIOD_MS)) {
    return false;
  }

  memset(out, 0, sizeof(*out));

  out->type = MSG_CBBA_STATE;
  out->src_id = appNodeIdFromIndex((uint8_t)(s->agent_id - 1u));
  out->seq = ++s->my_seq;
  out->exec_task = s->exec_task;
  out->done_mask = s->done_mask;

  for (t = 0u; t < TASK_MAX; t++) {
    out->bid[t] = s->bid_q[t];
    out->winner[t] = s->winner[t];
  }

  for (ai = 0u; ai < AGENT_COUNT; ai++) {
    out->agent_seq[ai] = s->last_seq[ai];
  }

  s->last_cbba_tx_ms = now_ms;
  return true;
}

void Cbba_MarkReachedDone(CbbaState* s, uint32_t now_ms) {
  const uint8_t t = s->exec_task;
  uint8_t i = 0u;

  if (t >= s->task_count) {
    return;
  }

  if (s->done[t] != 0u) {
    return;
  }

  /* 내가 winner 가 아닌 task 는 done 표시하지 않음 */
  if (s->winner[t] != s->agent_id) {
    return;
  }

  if (dist_m(s->self_pos, s->tasks[t].pos) <= DONE_RADIUS_M) {
    if (s->done_enter_ms[t] == 0u) {
      s->done_enter_ms[t] = now_ms;
    }

    if ((now_ms - s->done_enter_ms[t]) >= DONE_DWELL_MS) {
      s->done[t] = 1u;
      s->done_mask |= (uint8_t)(1u << t);
      s->winner[t] = 0u;
      s->bid_q[t] = 0;

      pathRemove(s, t);
      bundleRemove(s, t);

      for (i = 0u; i < s->task_count; i++) {
        s->done_enter_ms[i] = 0u;
      }

      updateLocalState(s);

      if (s->done_count < s->task_count) {
        addBundleTasks(s);
        updateLocalState(s);
      }

      s->replan_hold_until_ms = now_ms + REPLAN_HOLD_MS;

      if ((s->mission_done_since_ms == 0u) && (s->done_count >= s->task_count)) {
        s->mission_done_since_ms = now_ms;
      }

      DEBUG_PRINT("[DONE_LOCAL] agent=%u task=%u done_count=%u next_exec=%u\n",
                  (unsigned)s->agent_id, (unsigned)t, (unsigned)s->done_count,
                  (unsigned)s->exec_task);
    }
  } else {
    s->done_enter_ms[t] = 0u;
  }
}

/* done[] 배열 기반 완료 카운트 반환 (peer done_mask 반영 이후 최신값) */
uint8_t Cbba_GetGlobalDoneCount(const CbbaState* s) {
  return s->done_count;
}
