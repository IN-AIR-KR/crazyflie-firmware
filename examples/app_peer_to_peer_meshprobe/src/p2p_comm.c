#include "p2p_comm.h"

#include <string.h>

#include "radiolink.h"
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "ids.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"

typedef struct
{
  uint8_t src_id;
  uint8_t type;
  uint8_t seq;
} seen_t;

static uint8_t g_my_id = 0u;
static seen_t g_seen[SEEN_N];
static uint8_t g_seen_wr = 0u;

static app_rx_event_t g_rx_q[RX_QUEUE_N];
static uint8_t g_rx_head = 0u;
static uint8_t g_rx_tail = 0u;
static uint8_t g_rx_count_q = 0u;

static uint32_t g_last_rx_ms[AGENT_COUNT];
static uint32_t g_rx_count = 0u;
static uint32_t g_drop_count = 0u;

static P2PPacket g_txp;

static bool seenHas(uint8_t src, uint8_t type, uint8_t seq)
{
  uint8_t i = 0u;
  for (i = 0u; i < SEEN_N; i++)
  {
    if ((g_seen[i].src_id == src) &&
        (g_seen[i].type == type) &&
        (g_seen[i].seq == seq))
    {
      return true;
    }
  }
  return false;
}

static void seenPut(uint8_t src, uint8_t type, uint8_t seq)
{
  g_seen[g_seen_wr].src_id = src;
  g_seen[g_seen_wr].type = type;
  g_seen[g_seen_wr].seq = seq;

  g_seen_wr++;
  if (g_seen_wr >= SEEN_N)
  {
    g_seen_wr = 0u;
  }
}

static void rxQueuePush(const app_rx_event_t *e)
{
  if (g_rx_count_q >= RX_QUEUE_N)
  {
    g_drop_count++;
    return;
  }

  g_rx_q[g_rx_tail] = *e;
  g_rx_tail++;
  if (g_rx_tail >= RX_QUEUE_N)
  {
    g_rx_tail = 0u;
  }
  g_rx_count_q++;
}

static void p2pRxCb(P2PPacket *p)
{
  const uint8_t *b = (const uint8_t*)0;
  uint8_t type = 0u;
  uint8_t src  = 0u;
  uint8_t seq  = 0u;

  if ((p == (P2PPacket*)0) || (p->size < 6u))
  {
    return;
  }

  b = (const uint8_t*)p->data;
  type = b[0];
  src  = b[1];
  seq  = b[3];

  if (src == g_my_id) { return; }
  if (!appIsValidNodeId(src)) { return; }

  if (seenHas(src, type, seq))
  {
    return;
  }
  seenPut(src, type, seq);

  g_rx_count++;
  g_last_rx_ms[appNodeIndexFromId(src)] =
      (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  if ((type == MSG_CLAIM) && (p->size == sizeof(msg_claim_t)))
  {
    app_rx_event_t e;
    e.type = MSG_CLAIM;
    memcpy(&e.u.claim, p->data, sizeof(msg_claim_t));
    rxQueuePush(&e);
    return;
  }

  if ((type == MSG_DONE) && (p->size == sizeof(msg_done_t)))
  {
    app_rx_event_t e;
    e.type = MSG_DONE;
    memcpy(&e.u.done, p->data, sizeof(msg_done_t));
    rxQueuePush(&e);
    return;
  }

  if ((type == MSG_SNAPSHOT_FR) && (p->size == sizeof(msg_snapshot_frag_t)))
  {
    app_rx_event_t e;
    e.type = MSG_SNAPSHOT_FR;
    memcpy(&e.u.snapf, p->data, sizeof(msg_snapshot_frag_t));
    rxQueuePush(&e);
    return;
  }
}

void p2pCommInit(uint8_t my_radio_id)
{
  memset(g_seen, 0, sizeof(g_seen));
  memset(g_rx_q, 0, sizeof(g_rx_q));
  memset(g_last_rx_ms, 0, sizeof(g_last_rx_ms));

  g_my_id = my_radio_id;
  g_seen_wr = 0u;
  g_rx_head = 0u;
  g_rx_tail = 0u;
  g_rx_count_q = 0u;
  g_rx_count = 0u;
  g_drop_count = 0u;

  p2pRegisterCB(p2pRxCb);

  DEBUG_PRINT("[P2P_INIT] me=%s direct-only\n", appNodeName(g_my_id));
}

static void sendPacket(const void *buf, uint8_t sz)
{
  g_txp.size = sz;
  memcpy(g_txp.data, buf, sz);
  radiolinkSendP2PPacketBroadcast(&g_txp);
}

void p2pCommSendBeacon(const msg_beacon_t *m)
{
  if (m == (const msg_beacon_t*)0) { return; }
  sendPacket(m, (uint8_t)sizeof(*m));
}

void p2pCommSendClaim(const msg_claim_t *m)
{
  if (m == (const msg_claim_t*)0) { return; }
  sendPacket(m, (uint8_t)sizeof(*m));
}

void p2pCommSendDone(const msg_done_t *m)
{
  if (m == (const msg_done_t*)0) { return; }
  sendPacket(m, (uint8_t)sizeof(*m));
}

void p2pCommSendSnapshotFrag(const msg_snapshot_frag_t *m)
{
  if (m == (const msg_snapshot_frag_t*)0) { return; }
  sendPacket(m, (uint8_t)sizeof(*m));
}

bool p2pCommPollEvent(app_rx_event_t *out_evt)
{
  if ((out_evt == (app_rx_event_t*)0) || (g_rx_count_q == 0u))
  {
    return false;
  }

  *out_evt = g_rx_q[g_rx_head];
  g_rx_head++;
  if (g_rx_head >= RX_QUEUE_N)
  {
    g_rx_head = 0u;
  }
  g_rx_count_q--;
  return true;
}

uint32_t p2pCommGetLastRxMs(uint8_t peer_radio_id)
{
  if (!appIsValidNodeId(peer_radio_id))
  {
    return 0u;
  }
  return g_last_rx_ms[appNodeIndexFromId(peer_radio_id)];
}

uint32_t p2pCommGetRxCount(void)
{
  return g_rx_count;
}

uint32_t p2pCommGetDropCount(void)
{
  return g_drop_count;
}
