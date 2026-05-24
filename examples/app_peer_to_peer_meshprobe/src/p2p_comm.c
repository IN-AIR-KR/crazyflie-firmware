#include "p2p_comm.h"

#include <string.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "app.h"
#include "app_config.h"
#include "crtp.h"
#include "ids.h"
#include "radiolink.h"
#include "task.h"

#define CRTP_PORT_P2P_PROXY 0x09

#define DEBUG_MODULE "P2P"
#include "debug.h"

typedef struct {
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
static uint32_t g_last_beacon_rx_ms[AGENT_COUNT];
static uint8_t g_last_beacon_ready[AGENT_COUNT];
static uint8_t g_last_beacon_started[AGENT_COUNT];
static uint8_t g_last_beacon_state[AGENT_COUNT];

static uint32_t g_rx_count = 0u;
static uint32_t g_drop_count = 0u;

static float g_my_x_m = 0.0f;
static float g_my_y_m = 0.0f;

static P2PPacket g_txp;

static bool seenHas(uint8_t src, uint8_t type, uint8_t seq) {
  uint8_t i = 0u;

  for (i = 0u; i < SEEN_N; i++) {
    if ((g_seen[i].src_id == src) && (g_seen[i].type == type) &&
        (g_seen[i].seq == seq)) {
      return true;
    }
  }

  return false;
}

static void seenPut(uint8_t src, uint8_t type, uint8_t seq) {
  g_seen[g_seen_wr].src_id = src;
  g_seen[g_seen_wr].type = type;
  g_seen[g_seen_wr].seq = seq;

  g_seen_wr++;
  if (g_seen_wr >= SEEN_N) {
    g_seen_wr = 0u;
  }
}

static void rxQueuePush(const app_rx_event_t* e) {
  if (e == (const app_rx_event_t*)0) {
    return;
  }

  if (g_rx_count_q >= RX_QUEUE_N) {
    g_drop_count++;
    return;
  }

  g_rx_q[g_rx_tail] = *e;

  g_rx_tail++;
  if (g_rx_tail >= RX_QUEUE_N) {
    g_rx_tail = 0u;
  }

  g_rx_count_q++;
}

#if (USE_RANGE_LIMIT && (USE_BEACON_RANGE_LIMIT || USE_CBBA_RANGE_LIMIT))
static bool isWithinRadiusCm(int16_t tx_x_cm, int16_t tx_y_cm,
                             float radius_m) {
  const float tx_x_m = (float)tx_x_cm * 0.01f;
  const float tx_y_m = (float)tx_y_cm * 0.01f;
  const float dx = tx_x_m - g_my_x_m;
  const float dy = tx_y_m - g_my_y_m;
  const float r2 = radius_m * radius_m;

  return ((dx * dx) + (dy * dy)) <= r2;
}
#endif

static bool packetPassesRangeGate(uint8_t type, const P2PPacket* p) {
#if !(USE_RANGE_LIMIT && (USE_BEACON_RANGE_LIMIT || USE_CBBA_RANGE_LIMIT))
  (void)type;
  (void)p;
#endif
#if USE_RANGE_LIMIT
  if (type == MSG_BEACON) {
#if USE_BEACON_RANGE_LIMIT
    if ((p != (const P2PPacket*)0) && (p->size == sizeof(msg_beacon_t))) {
      const msg_beacon_t* m = (const msg_beacon_t*)p->data;
      return isWithinRadiusCm(m->tx_x_cm, m->tx_y_cm, BEACON_RADIUS_M);
    }
#endif
    return true;
  }

  if (type == MSG_SNAPSHOT_FR) {
#if USE_CBBA_RANGE_LIMIT
    if ((p != (const P2PPacket*)0) &&
        (p->size == sizeof(msg_snapshot_frag_t))) {
      const msg_snapshot_frag_t* m = (const msg_snapshot_frag_t*)p->data;
      return isWithinRadiusCm(m->tx_x_cm, m->tx_y_cm, CBBA_COMM_RADIUS_M);
    }
#endif
    return true;
  }
#endif

  return true;
}

static void p2pRxCb(P2PPacket* p) {
  const uint8_t* b = (const uint8_t*)0;
  uint8_t type = 0u;
  uint8_t src = 0u;
  uint8_t tx = 0u;
  uint8_t seq = 0u;
  uint32_t nowMs = 0u;

  if ((p == (P2PPacket*)0) || (p->size < 4u)) {
    return;
  }

  b = (const uint8_t*)p->data;
  type = b[0];
  src = b[1];
  tx = b[2];
  seq = b[3];

  if (src == g_my_id) {
    return;
  }

  if (!appIsValidNodeId(src)) {
    return;
  }

  if (!appIsValidNodeId(tx)) {
    return;
  }

  if (seenHas(src, type, seq)) {
    return;
  }

  if (!packetPassesRangeGate(type, p)) {
    g_drop_count++;
    return;
  }

  seenPut(src, type, seq);

  nowMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  g_rx_count++;
  g_last_rx_ms[appNodeIndexFromId(src)] = nowMs;

  if ((type == MSG_BEACON) && (p->size == sizeof(msg_beacon_t))) {
    static CRTPPacket crtp_pkt;

    {
      const uint8_t src_idx = appNodeIndexFromId(src);
      const msg_beacon_t* bm = (const msg_beacon_t*)p->data;

      g_last_beacon_rx_ms[src_idx] = nowMs;
      g_last_beacon_ready[src_idx] = bm->cbba_ready;
      g_last_beacon_started[src_idx] = bm->cbba_started;
      g_last_beacon_state[src_idx] = bm->app_state;
    }

    crtp_pkt.header = CRTP_HEADER(CRTP_PORT_P2P_PROXY, 0);
    crtp_pkt.size = (uint8_t)offsetof(msg_beacon_t, app_state);
    memcpy(crtp_pkt.data, p->data, crtp_pkt.size);
    crtpSendPacket(&crtp_pkt);
    return;
  }

  if ((type == MSG_SNAPSHOT_FR) && (p->size == sizeof(msg_snapshot_frag_t))) {
    app_rx_event_t e;
    e.type = MSG_SNAPSHOT_FR;
    memcpy(&e.u.snapf, p->data, sizeof(msg_snapshot_frag_t));
    rxQueuePush(&e);
    return;
  }
}

void p2pCommInit(uint8_t my_radio_id) {
  memset(g_seen, 0, sizeof(g_seen));
  memset(g_rx_q, 0, sizeof(g_rx_q));
  memset(g_last_rx_ms, 0, sizeof(g_last_rx_ms));
  memset(g_last_beacon_rx_ms, 0, sizeof(g_last_beacon_rx_ms));
  memset(g_last_beacon_ready, 0, sizeof(g_last_beacon_ready));
  memset(g_last_beacon_started, 0, sizeof(g_last_beacon_started));
  memset(g_last_beacon_state, 0, sizeof(g_last_beacon_state));

  g_my_id = my_radio_id;
  g_seen_wr = 0u;
  g_rx_head = 0u;
  g_rx_tail = 0u;
  g_rx_count_q = 0u;
  g_rx_count = 0u;
  g_drop_count = 0u;
  g_my_x_m = 0.0f;
  g_my_y_m = 0.0f;

  p2pRegisterCB(p2pRxCb);

  DEBUG_PRINT(
      "[P2P_INIT] me=%s mode=p2p_only beacon_range=%u beacon_radius=%.2f "
      "cbba_range=%u cbba_radius=%.2f packet_forward=0 legacy_ttl_hop=1\n",
      appNodeName(g_my_id), (unsigned)USE_BEACON_RANGE_LIMIT,
      (double)BEACON_RADIUS_M, (unsigned)USE_CBBA_RANGE_LIMIT,
      (double)CBBA_COMM_RADIUS_M);
}

static void sendPacket(const void* buf, uint8_t sz) {
  if ((buf == (const void*)0) || (sz == 0u)) {
    return;
  }

  g_txp.size = sz;
  memcpy(g_txp.data, buf, sz);
  radiolinkSendP2PPacketBroadcast(&g_txp);
}

void p2pCommSendBeacon(const msg_beacon_t* m) {
  if (m == (const msg_beacon_t*)0) {
    return;
  }

  sendPacket(m, (uint8_t)sizeof(*m));

  {
    static CRTPPacket crtp_tx;
    crtp_tx.header = CRTP_HEADER(CRTP_PORT_P2P_PROXY, 1);
    crtp_tx.size = (uint8_t)offsetof(msg_beacon_t, app_state);
    memcpy(crtp_tx.data, m, crtp_tx.size);
    crtpSendPacket(&crtp_tx);
  }
}

void p2pCommSendSnapshotFrag(const msg_snapshot_frag_t* m) {
  if (m == (const msg_snapshot_frag_t*)0) {
    return;
  }

  sendPacket(m, (uint8_t)sizeof(*m));
}

bool p2pCommPollEvent(app_rx_event_t* out_evt) {
  if ((out_evt == (app_rx_event_t*)0) || (g_rx_count_q == 0u)) {
    return false;
  }

  *out_evt = g_rx_q[g_rx_head];

  g_rx_head++;
  if (g_rx_head >= RX_QUEUE_N) {
    g_rx_head = 0u;
  }

  g_rx_count_q--;
  return true;
}

uint32_t p2pCommGetLastRxMs(uint8_t peer_radio_id) {
  if (!appIsValidNodeId(peer_radio_id)) {
    return 0u;
  }

  return g_last_rx_ms[appNodeIndexFromId(peer_radio_id)];
}

uint32_t p2pCommGetLastBeaconRxMs(uint8_t peer_radio_id) {
  if (!appIsValidNodeId(peer_radio_id)) {
    return 0u;
  }

  return g_last_beacon_rx_ms[appNodeIndexFromId(peer_radio_id)];
}

uint8_t p2pCommGetLastBeaconReady(uint8_t peer_radio_id) {
  if (!appIsValidNodeId(peer_radio_id)) {
    return 0u;
  }

  return g_last_beacon_ready[appNodeIndexFromId(peer_radio_id)];
}

uint8_t p2pCommGetLastBeaconStarted(uint8_t peer_radio_id) {
  if (!appIsValidNodeId(peer_radio_id)) {
    return 0u;
  }

  return g_last_beacon_started[appNodeIndexFromId(peer_radio_id)];
}

uint8_t p2pCommGetLastBeaconState(uint8_t peer_radio_id) {
  if (!appIsValidNodeId(peer_radio_id)) {
    return 0u;
  }

  return g_last_beacon_state[appNodeIndexFromId(peer_radio_id)];
}

uint32_t p2pCommGetRxCount(void) { return g_rx_count; }

uint32_t p2pCommGetDropCount(void) { return g_drop_count; }

void p2pCommSetLocalPos(float x_m, float y_m) {
  g_my_x_m = x_m;
  g_my_y_m = y_m;
}
