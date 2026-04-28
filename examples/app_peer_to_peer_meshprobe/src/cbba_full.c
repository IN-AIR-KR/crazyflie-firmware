#include "cbba_full.h"

#include <string.h>
#include <math.h>

#include "ids.h"

#define DEBUG_MODULE "CBBA"
#include "debug.h"

static float dist_m(CbbaVec2 a, CbbaVec2 b)
{
  const float dx = a.x_m - b.x_m;
  const float dy = a.y_m - b.y_m;
  return sqrtf((dx * dx) + (dy * dy));
}

static int16_t qbid_from_delta(float delta_m)
{
  int32_t q = (int32_t)lrintf(10000.0f - (delta_m * 1000.0f));

  if (q > 32767)
  {
    q = 32767;
  }

  if (q < -32768)
  {
    q = -32768;
  }

  return (int16_t)q;
}

static uint8_t bundleContains(const CbbaState *s, uint8_t task_id)
{
  uint8_t i = 0u;

  for (i = 0u; i < s->bundle_len; i++)
  {
    if (s->bundle[i] == task_id)
    {
      return 1u;
    }
  }

  return 0u;
}

static uint8_t pathFindIndex(const CbbaState *s, uint8_t task_id)
{
  uint8_t i = 0u;

  for (i = 0u; i < s->path_len; i++)
  {
    if (s->path[i] == task_id)
    {
      return i;
    }
  }

  return 255u;
}

static uint8_t bundleFindIndex(const CbbaState *s, uint8_t task_id)
{
  uint8_t i = 0u;

  for (i = 0u; i < s->bundle_len; i++)
  {
    if (s->bundle[i] == task_id)
    {
      return i;
    }
  }

  return 255u;
}

static uint8_t taskInPath(const CbbaState *s, uint8_t task_id)
{
  return (pathFindIndex(s, task_id) != 255u) ? 1u : 0u;
}

static uint8_t taskInBundle(const CbbaState *s, uint8_t task_id)
{
  return (bundleFindIndex(s, task_id) != 255u) ? 1u : 0u;
}

static void pathRemove(CbbaState *s, uint8_t task_id)
{
  uint8_t idx = pathFindIndex(s, task_id);
  uint8_t i = 0u;

  if (idx == 255u)
  {
    return;
  }

  for (i = idx; i + 1u < s->path_len; i++)
  {
    s->path[i] = s->path[i + 1u];
  }

  if (s->path_len > 0u)
  {
    s->path_len--;
  }
}

static void bundleRemove(CbbaState *s, uint8_t task_id)
{
  uint8_t idx = bundleFindIndex(s, task_id);
  uint8_t i = 0u;

  if (idx == 255u)
  {
    return;
  }

  for (i = idx; i + 1u < s->bundle_len; i++)
  {
    s->bundle[i] = s->bundle[i + 1u];
  }

  if (s->bundle_len > 0u)
  {
    s->bundle_len--;
  }
}

static void pathInsert(CbbaState *s, uint8_t task_id, uint8_t ins_idx)
{
  uint8_t i = 0u;

  if (s->path_len >= s->bundle_limit)
  {
    return;
  }

  for (i = s->path_len; i > ins_idx; i--)
  {
    s->path[i] = s->path[i - 1u];
  }

  s->path[ins_idx] = task_id;
  s->path_len++;
}

static float insertionDeltaCost(const CbbaState *s, uint8_t task_id, uint8_t ins_idx)
{
  CbbaVec2 prev_p;
  CbbaVec2 next_p;
  const CbbaVec2 task_p = s->tasks[task_id].pos;

  if (ins_idx == 0u)
  {
    prev_p = s->self_pos;
  }
  else
  {
    prev_p = s->tasks[s->path[ins_idx - 1u]].pos;
  }

  if (ins_idx >= s->path_len)
  {
    return dist_m(prev_p, task_p);
  }

  next_p = s->tasks[s->path[ins_idx]].pos;

  return dist_m(prev_p, task_p)
       + dist_m(task_p, next_p)
       - dist_m(prev_p, next_p);
}

static int16_t bestInsertionBid(const CbbaState *s, uint8_t task_id, uint8_t *best_idx)
{
  float best_delta = 1.0e9f;
  uint8_t i = 0u;
  uint8_t idx_best = 0u;

  for (i = 0u; i <= s->path_len; i++)
  {
    const float d = insertionDeltaCost(s, task_id, i);

    if (d < best_delta)
    {
      best_delta = d;
      idx_best = i;
    }
  }

  *best_idx = idx_best;
  return qbid_from_delta(best_delta);
}

static void updateLocalFp(CbbaState *s)
{
  uint8_t i = 0u;
  uint32_t h = 5381u;
  uint8_t done_cnt = 0u;
  uint8_t contested = 0u;

  for (i = 0u; i < s->task_count; i++)
  {
    h = ((h << 5u) + h) + (uint32_t)s->done[i];
    h = ((h << 5u) + h) + (uint32_t)s->winner[i];
    h = ((h << 5u) + h) + (uint32_t)((uint16_t)s->bid_q[i]);
    h = ((h << 5u) + h) + (uint32_t)s->ver[i];

    if (s->done[i] != 0u)
    {
      done_cnt++;
    }
    else if (s->winner[i] == 0u)
    {
      contested++;
    }
  }

  s->local_fp = h;
  s->done_count = done_cnt;
  s->contested_count = contested;
  s->exec_task = (s->path_len > 0u) ? s->path[0] : 255u;
}

static void releaseSuffix(CbbaState *s, uint8_t start_idx)
{
  uint8_t i = 0u;

  if (start_idx >= s->bundle_len)
  {
    return;
  }

  for (i = start_idx; i < s->bundle_len; i++)
  {
    const uint8_t t = s->bundle[i];

    if (t < s->task_count)
    {
      if (s->winner[t] == s->agent_id)
      {
        s->winner[t] = 0u;
        s->bid_q[t] = 0;
        s->ver[t]++;
      }

      pathRemove(s, t);
    }
  }

  s->bundle_len = start_idx;
}

static void addBundleTasks(CbbaState *s)
{
  while (s->bundle_len < s->bundle_limit)
  {
    uint8_t t = 0u;
    uint8_t best_t = 255u;
    uint8_t best_ins = 0u;
    int16_t best_bid = -32768;

    for (t = 0u; t < s->task_count; t++)
    {
      uint8_t ins_idx = 0u;
      int16_t my_bid = 0;

      if (!s->tasks[t].active)
      {
        continue;
      }

      if (s->done[t] != 0u)
      {
        continue;
      }

      if (bundleContains(s, t) != 0u)
      {
        continue;
      }

      my_bid = bestInsertionBid(s, t, &ins_idx);

      if ((s->winner[t] == 0u) ||
          (my_bid > s->bid_q[t]) ||
          ((my_bid == s->bid_q[t]) && (s->agent_id < s->winner[t])))
      {
        if (my_bid > best_bid)
        {
          best_bid = my_bid;
          best_t = t;
          best_ins = ins_idx;
        }
      }
    }

    if (best_t == 255u)
    {
      break;
    }

    s->bundle[s->bundle_len] = best_t;
    s->bundle_len++;

    pathInsert(s, best_t, best_ins);

    s->winner[best_t] = s->agent_id;
    s->bid_q[best_t] = best_bid;
    s->ver[best_t]++;
  }
}

static void initTasks(CbbaTask *tasks)
{
  uint8_t i = 0u;

  for (i = 0u; i < TASK_MAX; i++)
  {
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
}

void Cbba_Init(CbbaState *s, uint8_t agent_id, CbbaVec2 start_pos)
{
  uint8_t i = 0u;

  memset(s, 0, sizeof(*s));

  s->agent_id = agent_id;
  s->task_count = TASK_COUNT_RUNTIME;
  s->bundle_limit = BUNDLE_LIMIT;
  s->self_pos = start_pos;
  s->exec_task = 255u;
  s->replan_hold_until_ms = 0u;
  s->mission_done_since_ms = 0u;
  s->claim_rr = 0u;
  s->snap_frag_rr = 0u;

  initTasks(s->tasks);

  for (i = 0u; i < TASK_MAX; i++)
  {
    s->winner[i] = 0u;
    s->bid_q[i] = 0;
    s->ver[i] = 0u;
    s->done[i] = 0u;
    s->done_enter_ms[i] = 0u;
  }

  addBundleTasks(s);
  updateLocalFp(s);
}

void Cbba_SetPose(CbbaState *s, CbbaVec2 pos)
{
  s->self_pos = pos;
}

void Cbba_HandleClaim(CbbaState *s, const msg_claim_t *m)
{
  const uint8_t t = m->task_id;
  const uint8_t sender_agent = appAgentIdFromRadioLow(m->src_id);

  if (t >= s->task_count)
  {
    return;
  }

  if (s->done[t] != 0u)
  {
    return;
  }

  if ((m->ver > s->ver[t]) ||
      ((m->ver == s->ver[t]) &&
       ((m->bid_q > s->bid_q[t]) ||
        ((m->bid_q == s->bid_q[t]) && (sender_agent < s->winner[t])))))
  {
    s->winner[t] = sender_agent;
    s->bid_q[t] = m->bid_q;
    s->ver[t] = m->ver;
  }
}

void Cbba_HandleDone(CbbaState *s, const msg_done_t *m)
{
  const uint8_t t = m->task_id;

  if (t >= s->task_count)
  {
    return;
  }

  if (m->ver >= s->ver[t])
  {
    s->done[t] = 1u;
    s->winner[t] = 0u;
    s->bid_q[t] = 0;
    s->ver[t] = m->ver;

    pathRemove(s, t);
    bundleRemove(s, t);
    updateLocalFp(s);
  }
}

void Cbba_HandleSnapshotFrag(CbbaState *s, const msg_snapshot_frag_t *m)
{
  uint8_t k = 0u;

  for (k = 0u; k < m->task_count_in_frag; k++)
  {
    const uint8_t t = (uint8_t)(m->task_start_idx + k);
    const uint8_t in_done = (uint8_t)((m->done_mask_local >> k) & 0x01u);
    const uint8_t in_w = m->winner[k];
    const int16_t in_b = m->bid_q[k];
    const uint8_t in_v = m->ver[k];

    if (t >= s->task_count)
    {
      continue;
    }

    if ((in_v > s->ver[t]) ||
        ((in_v == s->ver[t]) &&
         ((in_done > s->done[t]) ||
          ((in_done == s->done[t]) &&
           ((in_b > s->bid_q[t]) ||
            ((in_b == s->bid_q[t]) && (in_w != 0u) && (in_w < s->winner[t])))))))
    {
      s->done[t] = in_done;
      s->winner[t] = in_done ? 0u : in_w;
      s->bid_q[t] = in_done ? 0 : in_b;
      s->ver[t] = in_v;

      if (in_done != 0u)
      {
        pathRemove(s, t);
        bundleRemove(s, t);
      }
    }
  }

  updateLocalFp(s);
}

void Cbba_LocalStep(CbbaState *s, uint32_t now_ms)
{
  uint8_t i = 0u;
  (void)now_ms;

  for (i = 0u; i < s->bundle_len; i++)
  {
    const uint8_t t = s->bundle[i];

    if ((t < s->task_count) && (s->winner[t] != s->agent_id))
    {
      releaseSuffix(s, i);
      break;
    }
  }

  if ((s->path_len == 0u || s->bundle_len < s->bundle_limit) &&
      (s->done_count < s->task_count))
  {
    addBundleTasks(s);
  }

  updateLocalFp(s);
}

void Cbba_MarkReachedDone(CbbaState *s, uint32_t now_ms)
{
  const uint8_t t = s->exec_task;
  uint8_t i = 0u;

  if (t >= s->task_count)
  {
    return;
  }

  if (s->done[t] != 0u)
  {
    return;
  }

  if (dist_m(s->self_pos, s->tasks[t].pos) <= DONE_RADIUS_M)
  {
    if (s->done_enter_ms[t] == 0u)
    {
      s->done_enter_ms[t] = now_ms;
    }

    if ((now_ms - s->done_enter_ms[t]) >= DONE_DWELL_MS)
    {
      s->done[t] = 1u;
      s->winner[t] = 0u;
      s->bid_q[t] = 0;
      s->ver[t]++;

      pathRemove(s, t);
      bundleRemove(s, t);

      for (i = 0u; i < s->task_count; i++)
      {
        s->done_enter_ms[i] = 0u;
      }

      updateLocalFp(s);

      if (s->done_count < s->task_count)
      {
        addBundleTasks(s);
        updateLocalFp(s);
      }

      s->replan_hold_until_ms = now_ms + REPLAN_HOLD_MS;

      if (s->done_count >= s->task_count)
      {
        if (s->mission_done_since_ms == 0u)
        {
          s->mission_done_since_ms = now_ms;
        }
      }

      DEBUG_PRINT("[DONE_LOCAL] agent=%u task=%u done_count=%u next_exec=%u\n",
                  (unsigned)s->agent_id,
                  (unsigned)t,
                  (unsigned)s->done_count,
                  (unsigned)s->exec_task);
    }
  }
  else
  {
    s->done_enter_ms[t] = 0u;
  }
}

bool Cbba_MakeClaimMsg(CbbaState *s, uint32_t now_ms, msg_claim_t *out)
{
  uint8_t t = 255u;
  uint8_t rr_used = 0u;

  if ((out == (msg_claim_t*)0) || (s->bundle_len == 0u))
  {
    return false;
  }

  if ((s->last_claim_tx_ms != 0u) &&
      ((now_ms - s->last_claim_tx_ms) < CLAIM_TX_PERIOD_MS))
  {
    return false;
  }

  rr_used = s->claim_rr;

  if (rr_used >= s->bundle_len)
  {
    rr_used = 0u;
  }

  t = s->bundle[rr_used];

  s->claim_rr++;

  if (s->claim_rr >= s->bundle_len)
  {
    s->claim_rr = 0u;
  }

  memset(out, 0, sizeof(*out));

  out->type = MSG_CLAIM;
  out->src_id = appNodeIdFromIndex((uint8_t)(s->agent_id - 1u));
  out->tx_id = out->src_id;
  out->seq = ++s->tx_seq;
  out->ttl = 0u;
  out->hop = 0u;
  out->task_id = t;
  out->bid_q = s->bid_q[t];
  out->ver = s->ver[t];
  out->path_idx = rr_used;

  s->last_claim_tx_ms = now_ms;
  return true;
}

bool Cbba_MakeDoneMsg(CbbaState *s, uint32_t now_ms, msg_done_t *out)
{
  uint8_t t = 0u;

  if (out == (msg_done_t*)0)
  {
    return false;
  }

  if ((s->last_done_tx_ms != 0u) &&
      ((now_ms - s->last_done_tx_ms) < DONE_REPEAT_PERIOD_MS))
  {
    return false;
  }

  for (t = 0u; t < s->task_count; t++)
  {
    if ((s->done[t] != 0u) && (s->ver[t] != 0u))
    {
      memset(out, 0, sizeof(*out));

      out->type = MSG_DONE;
      out->src_id = appNodeIdFromIndex((uint8_t)(s->agent_id - 1u));
      out->tx_id = out->src_id;
      out->seq = ++s->tx_seq;
      out->ttl = 0u;
      out->hop = 0u;
      out->task_id = t;
      out->ver = s->ver[t];

      s->last_done_tx_ms = now_ms;
      return true;
    }
  }

  return false;
}

bool Cbba_MakeSnapshotFragMsg(CbbaState *s, uint32_t now_ms, msg_snapshot_frag_t *out)
{
  uint8_t frag_idx = 0u;
  uint8_t start_idx = 0u;
  uint8_t remain = 0u;
  uint8_t k = 0u;

  if (out == (msg_snapshot_frag_t*)0)
  {
    return false;
  }

  if ((s->last_snapshot_tx_ms != 0u) &&
      ((now_ms - s->last_snapshot_tx_ms) < SNAPSHOT_TX_PERIOD_MS))
  {
    return false;
  }

  frag_idx = s->snap_frag_rr;

  if (frag_idx >= SNAP_FRAG_COUNT)
  {
    frag_idx = 0u;
  }

  start_idx = (uint8_t)(frag_idx * SNAP_FRAG_TASKS);
  remain = (uint8_t)(s->task_count - start_idx);

  memset(out, 0, sizeof(*out));

  out->type = MSG_SNAPSHOT_FR;
  out->src_id = appNodeIdFromIndex((uint8_t)(s->agent_id - 1u));
  out->tx_id = out->src_id;
  out->seq = ++s->tx_seq;
  out->ttl = 0u;
  out->hop = 0u;

  out->frag_idx = frag_idx;
  out->frag_count = SNAP_FRAG_COUNT;
  out->task_start_idx = start_idx;
  out->task_count_total = s->task_count;
  out->task_count_in_frag = (remain > SNAP_FRAG_TASKS) ? SNAP_FRAG_TASKS : remain;
  out->exec_task = s->exec_task;

  for (k = 0u; k < out->task_count_in_frag; k++)
  {
    const uint8_t t = (uint8_t)(start_idx + k);

    out->done_mask_local |= (uint8_t)((s->done[t] & 0x01u) << k);
    out->winner[k] = s->winner[t];
    out->bid_q[k] = s->bid_q[t];
    out->ver[k] = s->ver[t];
  }

  s->snap_frag_rr++;

  if (s->snap_frag_rr >= SNAP_FRAG_COUNT)
  {
    s->snap_frag_rr = 0u;
  }

  s->last_snapshot_tx_ms = now_ms;
  return true;
}

void Cbba_InitPeerCache(PeerSnapshotCache *c)
{
  memset(c, 0, sizeof(*c));

  c->src_id = 0u;
  c->exec_task = 255u;
}

void Cbba_UpdatePeerCacheFromFrag(PeerSnapshotCache *c, const msg_snapshot_frag_t *m)
{
  uint8_t k = 0u;

  c->valid = 1u;
  c->src_id = m->src_id;
  c->frag_count = m->frag_count;
  c->task_count_total = m->task_count_total;
  c->exec_task = m->exec_task;
  c->got_mask |= (uint8_t)(1u << m->frag_idx);

  for (k = 0u; k < m->task_count_in_frag; k++)
  {
    const uint8_t t = (uint8_t)(m->task_start_idx + k);

    if (t >= TASK_MAX)
    {
      continue;
    }

    c->done[t] = (uint8_t)((m->done_mask_local >> k) & 0x01u);
    c->winner[t] = m->winner[k];
    c->bid_q[t] = m->bid_q[k];
    c->ver[t] = m->ver[k];
  }
}

static uint32_t fpFromArrays(const uint8_t *done,
                             const uint8_t *winner,
                             const int16_t *bid_q,
                             const uint8_t *ver,
                             uint8_t task_count)
{
  uint8_t i = 0u;
  uint32_t h = 5381u;

  for (i = 0u; i < task_count; i++)
  {
    h = ((h << 5u) + h) + (uint32_t)done[i];
    h = ((h << 5u) + h) + (uint32_t)winner[i];
    h = ((h << 5u) + h) + (uint32_t)((uint16_t)bid_q[i]);
    h = ((h << 5u) + h) + (uint32_t)ver[i];
  }

  return h;
}

void Cbba_GetObserverMetrics(const CbbaState *self,
                             const PeerSnapshotCache *peer1,
                             const PeerSnapshotCache *peer2,
                             CbbaObserverMetrics *out)
{
  uint8_t t = 0u;
  uint8_t need_mask = 0u;
  const uint8_t task_count = self->task_count;

  memset(out, 0, sizeof(*out));

  out->exec_shadow[0] = self->exec_task;
  out->exec_shadow[1] = peer1->exec_task;
  out->exec_shadow[2] = peer2->exec_task;

  if ((peer1->valid == 0u) || (peer2->valid == 0u))
  {
    out->all_known = 0u;
    out->fp_shadow[0] = fpFromArrays(self->done, self->winner, self->bid_q, self->ver, task_count);
    return;
  }

  need_mask = (uint8_t)((1u << SNAP_FRAG_COUNT) - 1u);

  if (((peer1->got_mask & need_mask) != need_mask) ||
      ((peer2->got_mask & need_mask) != need_mask))
  {
    out->all_known = 0u;
    out->fp_shadow[0] = fpFromArrays(self->done, self->winner, self->bid_q, self->ver, task_count);
    out->fp_shadow[1] = fpFromArrays(peer1->done, peer1->winner, peer1->bid_q, peer1->ver, task_count);
    out->fp_shadow[2] = fpFromArrays(peer2->done, peer2->winner, peer2->bid_q, peer2->ver, task_count);
    return;
  }

  out->all_known = 1u;
  out->fp_shadow[0] = fpFromArrays(self->done, self->winner, self->bid_q, self->ver, task_count);
  out->fp_shadow[1] = fpFromArrays(peer1->done, peer1->winner, peer1->bid_q, peer1->ver, task_count);
  out->fp_shadow[2] = fpFromArrays(peer2->done, peer2->winner, peer2->bid_q, peer2->ver, task_count);

  for (t = 0u; t < task_count; t++)
  {
    const uint8_t w0 = self->winner[t];
    const uint8_t w1 = peer1->winner[t];
    const uint8_t w2 = peer2->winner[t];

    if ((w0 == w1) && (w1 == w2))
    {
      out->equal_winner_tasks++;
    }
    else
    {
      out->contested_tasks++;
    }
  }

  out->winner_conv = (out->equal_winner_tasks == task_count) ? 1u : 0u;
  out->fp_conv =
      ((out->fp_shadow[0] == out->fp_shadow[1]) &&
       (out->fp_shadow[1] == out->fp_shadow[2])) ? 1u : 0u;
}

uint8_t Cbba_GetGlobalDoneCount(const CbbaState *self,
                                const PeerSnapshotCache *peer1,
                                const PeerSnapshotCache *peer2)
{
  uint8_t t = 0u;
  uint8_t cnt = 0u;

  for (t = 0u; t < self->task_count; t++)
  {
    const uint8_t d0 = self->done[t];
    const uint8_t d1 = ((peer1->valid != 0u) ? peer1->done[t] : 0u);
    const uint8_t d2 = ((peer2->valid != 0u) ? peer2->done[t] : 0u);

    if ((d0 != 0u) || (d1 != 0u) || (d2 != 0u))
    {
      cnt++;
    }
  }

  return cnt;
}

void Cbba_DebugPrintTables(const char *tag,
                           const CbbaState *self,
                           const PeerSnapshotCache *peer1,
                           const PeerSnapshotCache *peer2)
{
  uint8_t t = 0u;
  uint8_t i = 0u;

  DEBUG_PRINT("[CBBA_DUMP] tag=%s agent=%u exec=%u done_count=%u task_count=%u path_len=%u bundle_len=%u peer1_id=0x%02X peer2_id=0x%02X\n",
              tag,
              (unsigned)self->agent_id,
              (unsigned)self->exec_task,
              (unsigned)self->done_count,
              (unsigned)self->task_count,
              (unsigned)self->path_len,
              (unsigned)self->bundle_len,
              (unsigned)peer1->src_id,
              (unsigned)peer2->src_id);

  for (t = 0u; t < self->task_count; t++)
  {
    DEBUG_PRINT("[CBBA_TASK_SELF] t=%u act=%u done=%u winner=%u bid=%d ver=%u pos=(%.2f,%.2f) in_path=%u in_bundle=%u\n",
                (unsigned)t,
                (unsigned)(self->tasks[t].active ? 1u : 0u),
                (unsigned)self->done[t],
                (unsigned)self->winner[t],
                (int)self->bid_q[t],
                (unsigned)self->ver[t],
                (double)self->tasks[t].pos.x_m,
                (double)self->tasks[t].pos.y_m,
                (unsigned)taskInPath(self, t),
                (unsigned)taskInBundle(self, t));

    DEBUG_PRINT("[CBBA_TASK_PEER] t=%u P1(id=0x%02X,valid=%u,done=%u,winner=%u,bid=%d,ver=%u) P2(id=0x%02X,valid=%u,done=%u,winner=%u,bid=%d,ver=%u)\n",
                (unsigned)t,
                (unsigned)peer1->src_id,
                (unsigned)peer1->valid,
                (unsigned)peer1->done[t],
                (unsigned)peer1->winner[t],
                (int)peer1->bid_q[t],
                (unsigned)peer1->ver[t],
                (unsigned)peer2->src_id,
                (unsigned)peer2->valid,
                (unsigned)peer2->done[t],
                (unsigned)peer2->winner[t],
                (int)peer2->bid_q[t],
                (unsigned)peer2->ver[t]);
  }

  for (i = 0u; i < TASK_MAX; i++)
  {
    const uint8_t path_v = (i < self->path_len) ? self->path[i] : 255u;
    const uint8_t bundle_v = (i < self->bundle_len) ? self->bundle[i] : 255u;

    DEBUG_PRINT("[CBBA_PATH_BUNDLE] idx=%u path=%u bundle=%u\n",
                (unsigned)i,
                (unsigned)path_v,
                (unsigned)bundle_v);
  }
}
