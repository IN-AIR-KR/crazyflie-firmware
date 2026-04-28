#include "cbba_state.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "ids.h"

#define DEBUG_MODULE "CBBA"
#include "debug.h"

static const int16_t g_score_table[APP_NUM_DRONES][APP_NUM_TASKS] =
{
  { 95, 90, 85, 80, 75, 70, 65, 60, 55 }, /* D1 */
  { 93, 96, 88, 82, 78, 72, 66, 62, 58 }, /* D2 */
  { 91, 89, 97, 84, 79, 74, 68, 64, 61 }  /* D3 */
};

static bool isObserver(const cbba_state_t *s)
{
  return (s->id == APP_OBSERVER_ID);
}

static int16_t myScore(const cbba_state_t *s, uint8_t taskId)
{
  return g_score_table[s->my_idx][taskId];
}

static bool taskInBundle(const cbba_state_t *s, uint8_t taskId)
{
  uint8_t i = 0u;
  for (i = 0u; i < s->bundle_len; i++)
  {
    if (s->bundle[i] == taskId)
    {
      return true;
    }
  }
  return false;
}

static void packWinners(const uint8_t *owner, uint8_t packed[3])
{
  uint8_t t = 0u;
  uint32_t bits = 0u;

  for (t = 0u; t < APP_NUM_TASKS; t++)
  {
    uint8_t code = appWinnerCodeFromId(owner[t]);
    bits |= ((uint32_t)(code & 0x03u) << (2u * t));
  }

  packed[0] = (uint8_t)(bits & 0xFFu);
  packed[1] = (uint8_t)((bits >> 8u) & 0xFFu);
  packed[2] = (uint8_t)((bits >> 16u) & 0xFFu);
}

static uint8_t unpackWinner(const uint8_t packed[3], uint8_t taskId)
{
  uint32_t bits = 0u;
  bits |= (uint32_t)packed[0];
  bits |= ((uint32_t)packed[1] << 8u);
  bits |= ((uint32_t)packed[2] << 16u);
  return (uint8_t)((bits >> (2u * taskId)) & 0x03u);
}

static void packDone(const bool *done, uint8_t bits2[2])
{
  uint16_t bits = 0u;
  uint8_t t = 0u;

  for (t = 0u; t < APP_NUM_TASKS; t++)
  {
    if (done[t]) { bits |= (uint16_t)(1u << t); }
  }

  bits2[0] = (uint8_t)(bits & 0xFFu);
  bits2[1] = (uint8_t)((bits >> 8u) & 0xFFu);
}

static uint32_t tableFingerprint(const cbba_state_t *s)
{
  uint8_t t = 0u;
  uint32_t h = 5381u;

  for (t = 0u; t < APP_NUM_TASKS; t++)
  {
    h = ((h << 5u) + h) + (uint32_t)appWinnerCodeFromId(s->owner[t]);
    h = ((h << 5u) + h) + (uint32_t)((uint16_t)s->owner_bid[t]);
    h = ((h << 5u) + h) + (uint32_t)(s->done[t] ? 1u : 0u);
  }
  return h;
}

static void cbbaRefreshObserverSelf(cbba_state_t *s)
{
  uint8_t packed[3];
  uint8_t doneBits[2];
  const uint8_t meIdx = s->my_idx;

  if (!isObserver(s))
  {
    return;
  }

  packWinners(s->owner, packed);
  packDone(s->done, doneBits);

  s->seen_snapshot[meIdx] = true;
  s->fp_shadow[meIdx] = s->local_table_fp;
  s->packed_winners_shadow[meIdx][0] = packed[0];
  s->packed_winners_shadow[meIdx][1] = packed[1];
  s->packed_winners_shadow[meIdx][2] = packed[2];
  s->done_bits_shadow[meIdx][0] = doneBits[0];
  s->done_bits_shadow[meIdx][1] = doneBits[1];
  s->bundle_len_shadow[meIdx] = s->bundle_len;
  memcpy(s->bundle_shadow[meIdx], s->bundle, APP_BUNDLE_SIZE);
  s->exec_task_shadow[meIdx] = s->exec_task;
  s->last_evt_shadow[meIdx] = s->evt_id;
}

static void rebuildBundle(cbba_state_t *s)
{
  uint8_t oldBundle[APP_BUNDLE_SIZE];
  uint8_t oldLen = s->bundle_len;
  uint8_t i = 0u;
  uint8_t k = 0u;

  for (i = 0u; i < APP_BUNDLE_SIZE; i++)
  {
    oldBundle[i] = s->bundle[i];
  }

  for (i = 0u; i < APP_NUM_TASKS; i++)
  {
    if (s->owner[i] == s->id)
    {
      s->owner[i] = 0u;
      s->owner_bid[i] = 0;
    }
  }

  s->bundle_len = 0u;
  for (k = 0u; k < APP_BUNDLE_SIZE; k++)
  {
    int16_t bestScore = -32768;
    uint8_t bestTask = 0xFFu;
    uint8_t t = 0u;

    for (t = 0u; t < APP_NUM_TASKS; t++)
    {
      const int16_t myBid = myScore(s, t);
      const uint8_t curOwner = s->owner[t];
      const int16_t curBid = s->owner_bid[t];

      if (s->done[t]) { continue; }
      if (taskInBundle(s, t)) { continue; }

      if ((curOwner == 0u) ||
          (curOwner == s->id) ||
          (myBid > curBid) ||
          ((myBid == curBid) && (s->id < curOwner)))
      {
        if (myBid > bestScore)
        {
          bestScore = myBid;
          bestTask = t;
        }
      }
    }

    if (bestTask != 0xFFu)
    {
      s->bundle[s->bundle_len] = bestTask;
      s->bundle_len++;
      s->owner[bestTask] = s->id;
      s->owner_bid[bestTask] = myScore(s, bestTask);
    }
  }

  s->exec_task = (s->bundle_len > 0u) ? s->bundle[0] : 0xFFu;
  s->claim_rr = 0u;

  if ((oldLen != s->bundle_len) ||
      (memcmp(oldBundle, s->bundle, sizeof(oldBundle)) != 0))
  {
    s->evt_id++;
    s->exec_start_ms = 0u;
  }

  s->local_table_fp = tableFingerprint(s);
  cbbaRefreshObserverSelf(s);
}

void cbbaInit(cbba_state_t *s, uint8_t myId)
{
  memset(s, 0, sizeof(*s));

  s->id = myId;
  s->my_idx = appNodeIndexFromId(myId);
  s->exec_task = 0xFFu;

  rebuildBundle(s);
  cbbaRefreshObserverSelf(s);

  DEBUG_PRINT("[CBBA_INIT] me=%s observer=%u\n",
              appNodeName(s->id),
              (unsigned)isObserver(s));
}

void cbbaOnClaim(cbba_state_t *s, const msg_claim_t *m)
{
  const uint8_t task = m->task_id;
  const uint8_t src = m->src_id;
  const int16_t bid = m->bid_s16;

  if (task >= APP_NUM_TASKS) { return; }
  if (s->done[task]) { return; }

  if ((bid > s->owner_bid[task]) ||
      ((bid == s->owner_bid[task]) && (src < s->owner[task])))
  {
    s->owner[task] = src;
    s->owner_bid[task] = bid;
    s->apply_count++;

    if ((s->id != src) && taskInBundle(s, task))
    {
      rebuildBundle(s);
    }
    else
    {
      s->local_table_fp = tableFingerprint(s);
      cbbaRefreshObserverSelf(s);
    }
  }
  else
  {
    s->stale_count++;
  }
}

void cbbaOnDone(cbba_state_t *s, const msg_done_t *m)
{
  const uint8_t task = m->task_id;

  if (task >= APP_NUM_TASKS) { return; }
  if (s->done[task]) { return; }

  s->done[task] = true;
  s->owner[task] = 0u;
  s->owner_bid[task] = 0;
  s->evt_id++;

  rebuildBundle(s);
  cbbaRefreshObserverSelf(s);
}

void cbbaOnSnapshot(cbba_state_t *s, const msg_snapshot_t *m, uint32_t nowMs)
{
  const uint8_t originIdx = appNodeIndexFromId(m->src_id);
  const uint8_t evtIdx = (uint8_t)(m->evt_id % EVT_SLOTS);

  if (!isObserver(s))
  {
    return;
  }

  s->seen_snapshot[originIdx] = true;
  s->fp_shadow[originIdx] = m->table_fp;
  s->bundle_len_shadow[originIdx] = m->bundle_len;
  s->exec_task_shadow[originIdx] = m->exec_task;
  memcpy(s->packed_winners_shadow[originIdx], m->packed_winners, 3u);
  memcpy(s->done_bits_shadow[originIdx], m->done_bits, 2u);
  memcpy(s->bundle_shadow[originIdx], m->bundle, APP_BUNDLE_SIZE);

  if (!s->first_seen_evt_valid[originIdx][evtIdx])
  {
    s->first_seen_evt_valid[originIdx][evtIdx] = true;
    s->first_seen_evt_ms[originIdx][evtIdx] = nowMs;
  }

  s->last_evt_shadow[originIdx] = m->evt_id;
}

void cbbaStepLocal(cbba_state_t *s, uint32_t nowMs)
{
  if (s->exec_task != 0xFFu)
  {
    if (s->exec_start_ms == 0u)
    {
      s->exec_start_ms = nowMs;
    }
  }

  if ((s->exec_task != 0xFFu) &&
      ((nowMs - s->exec_start_ms) >= EXEC_DWELL_MS))
  {
    const uint8_t task = s->exec_task;
    s->done[task] = true;
    s->owner[task] = 0u;
    s->owner_bid[task] = 0;
    s->evt_id++;
    rebuildBundle(s);
  }

  s->local_table_fp = tableFingerprint(s);
  cbbaRefreshObserverSelf(s);
}

bool cbbaMakeClaim(cbba_state_t *s, msg_claim_t *outMsg)
{
  uint8_t task = 0xFFu;
  const uint32_t nowMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  if ((outMsg == (msg_claim_t*)0) || (s->bundle_len == 0u))
  {
    return false;
  }

  if ((s->last_claim_tx_ms != 0u) &&
      ((nowMs - s->last_claim_tx_ms) < CLAIM_TX_PERIOD_MS))
  {
    return false;
  }

  task = s->bundle[s->claim_rr];
  s->claim_rr++;
  if (s->claim_rr >= s->bundle_len)
  {
    s->claim_rr = 0u;
  }

  memset(outMsg, 0, sizeof(*outMsg));
  outMsg->type = MSG_CLAIM;
  outMsg->src_id = s->id;
  outMsg->tx_id = s->id;
  outMsg->seq = ++s->tx_seq;
  outMsg->ttl = MESH_TTL_INIT;
  outMsg->hop = 0u;
  outMsg->evt_id = s->evt_id;
  outMsg->task_id = task;
  outMsg->bundle_rank = s->claim_rr;
  outMsg->bid_s16 = myScore(s, task);

  s->last_claim_tx_ms = nowMs;
  return true;
}

bool cbbaMakeDone(cbba_state_t *s, msg_done_t *outMsg)
{
  static uint8_t last_done_evt_sent = 0u;

  if (outMsg == (msg_done_t*)0)
  {
    return false;
  }

  if (s->evt_id == last_done_evt_sent)
  {
    return false;
  }

  {
    uint8_t t = 0u;
    for (t = 0u; t < APP_NUM_TASKS; t++)
    {
      if (s->done[t])
      {
        memset(outMsg, 0, sizeof(*outMsg));
        outMsg->type = MSG_DONE;
        outMsg->src_id = s->id;
        outMsg->tx_id = s->id;
        outMsg->seq = ++s->tx_seq;
        outMsg->ttl = MESH_TTL_INIT;
        outMsg->hop = 0u;
        outMsg->evt_id = s->evt_id;
        outMsg->task_id = t;
        last_done_evt_sent = s->evt_id;
        return true;
      }
    }
  }

  return false;
}

bool cbbaMakeSnapshot(cbba_state_t *s, msg_snapshot_t *outMsg)
{
  uint8_t doneBits[2];
  const uint32_t nowMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  if (outMsg == (msg_snapshot_t*)0)
  {
    return false;
  }

  if ((s->last_snapshot_tx_ms != 0u) &&
      ((nowMs - s->last_snapshot_tx_ms) < SNAPSHOT_TX_PERIOD_MS))
  {
    return false;
  }

  memset(outMsg, 0, sizeof(*outMsg));
  outMsg->type = MSG_SNAPSHOT;
  outMsg->src_id = s->id;
  outMsg->tx_id = s->id;
  outMsg->seq = ++s->tx_seq;
  outMsg->ttl = MESH_TTL_INIT;
  outMsg->hop = 0u;
  outMsg->evt_id = s->evt_id;
  outMsg->exec_task = s->exec_task;
  outMsg->bundle_len = s->bundle_len;
  memcpy(outMsg->bundle, s->bundle, APP_BUNDLE_SIZE);

  packWinners(s->owner, outMsg->packed_winners);
  packDone(s->done, doneBits);
  outMsg->done_bits[0] = doneBits[0];
  outMsg->done_bits[1] = doneBits[1];
  outMsg->table_fp = s->local_table_fp;

  s->last_snapshot_tx_ms = nowMs;
  return true;
}

void cbbaPrintObserverSummary(cbba_state_t *s, uint32_t nowMs)
{
  uint8_t t = 0u;
  uint8_t equal = 0u;
  uint8_t contested = 0u;
  bool allKnown = true;
  bool wconv = false;
  bool fpconv = false;

  if ((nowMs - s->last_summary_ms) < SUMMARY_LOG_MS)
  {
    return;
  }
  s->last_summary_ms = nowMs;

  if (!isObserver(s))
  {
    return;
  }

  if (!(s->seen_snapshot[0] && s->seen_snapshot[1] && s->seen_snapshot[2]))
  {
    allKnown = false;
  }

  if (allKnown)
  {
    for (t = 0u; t < APP_NUM_TASKS; t++)
    {
      const uint8_t w0 = unpackWinner(s->packed_winners_shadow[0], t);
      const uint8_t w1 = unpackWinner(s->packed_winners_shadow[1], t);
      const uint8_t w2 = unpackWinner(s->packed_winners_shadow[2], t);

      if ((w0 == w1) && (w1 == w2))
      {
        equal++;
      }
      else
      {
        contested++;
      }
    }

    wconv = (equal == APP_NUM_TASKS);
    fpconv = ((s->fp_shadow[0] == s->fp_shadow[1]) &&
              (s->fp_shadow[1] == s->fp_shadow[2]));
  }

  DEBUG_PRINT("[OBS] relay=%d all_known=%u equal=%u contested=%u wconv=%u fpconv=%u fp=(%lu,%lu,%lu) apply=%lu stale=%lu\n",
              APP_USE_MESH_RELAY,
              (unsigned)allKnown,
              (unsigned)equal,
              (unsigned)contested,
              (unsigned)wconv,
              (unsigned)fpconv,
              (unsigned long)s->fp_shadow[0],
              (unsigned long)s->fp_shadow[1],
              (unsigned long)s->fp_shadow[2],
              (unsigned long)s->apply_count,
              (unsigned long)s->stale_count);
}
