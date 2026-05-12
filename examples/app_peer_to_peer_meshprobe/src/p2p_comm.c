#include "p2p_comm.h"

#include <string.h>

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
static uint32_t g_rx_count = 0u;
static uint32_t g_drop_count = 0u;

static float g_my_x_m = 0.0f;
static float g_my_y_m = 0.0f;

static P2PPacket g_txp;
#if USE_MESH
static P2PPacket g_relay_pkt;
#endif

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

static bool isWithinRadiusCm(int16_t tx_x_cm, int16_t tx_y_cm,
                             float radius_m) {
  const float tx_x_m = (float)tx_x_cm * 0.01f;
  const float tx_y_m = (float)tx_y_cm * 0.01f;
  const float dx = tx_x_m - g_my_x_m;
  const float dy = tx_y_m - g_my_y_m;
  const float r2 = radius_m * radius_m;

  return ((dx * dx) + (dy * dy)) <= r2;
}

static bool packetPassesRangeGate(uint8_t type, const P2PPacket* p) {
#if USE_RANGE_LIMIT
  if (type == MSG_BEACON) {
#if USE_BEACON_RANGE_LIMIT
    if (p->size == sizeof(msg_beacon_t)) {
      const msg_beacon_t* m = (const msg_beacon_t*)p->data;
      return isWithinRadiusCm(m->tx_x_cm, m->tx_y_cm, BEACON_RADIUS_M);
    }
#endif
    return true;
  }

  if (type == MSG_CLAIM) {
#if USE_CBBA_RANGE_LIMIT
    if (p->size == sizeof(msg_claim_t)) {
      const msg_claim_t* m = (const msg_claim_t*)p->data;
      return isWithinRadiusCm(m->tx_x_cm, m->tx_y_cm, CBBA_COMM_RADIUS_M);
    }
#endif
    return true;
  }

  if (type == MSG_DONE) {
#if USE_CBBA_RANGE_LIMIT
    if (p->size == sizeof(msg_done_t)) {
      const msg_done_t* m = (const msg_done_t*)p->data;
      return isWithinRadiusCm(m->tx_x_cm, m->tx_y_cm, CBBA_COMM_RADIUS_M);
    }
#endif
    return true;
  }

  if (type == MSG_SNAPSHOT_FR) {
#if USE_CBBA_RANGE_LIMIT
    if (p->size == sizeof(msg_snapshot_frag_t)) {
      const msg_snapshot_frag_t* m = (const msg_snapshot_frag_t*)p->data;
      return isWithinRadiusCm(m->tx_x_cm, m->tx_y_cm, CBBA_COMM_RADIUS_M);
    }
#endif
    return true;
  }

  if (type == MSG_DONE_LEDGER) {
#if USE_CBBA_RANGE_LIMIT
    if (p->size == sizeof(msg_done_ledger_t)) {
      const msg_done_ledger_t* m = (const msg_done_ledger_t*)p->data;
      return isWithinRadiusCm(m->tx_x_cm, m->tx_y_cm, CBBA_COMM_RADIUS_M);
    }
#endif
    return true;
  }
#endif

  return true;
}

#if USE_MESH
static void updateRelayTxPosition(uint8_t type, P2PPacket* pkt) {
  if ((pkt == (P2PPacket*)0) || (pkt->size < 6u)) {
    return;
  }

  if ((type == MSG_BEACON) && (pkt->size == sizeof(msg_beacon_t))) {
    msg_beacon_t* m = (msg_beacon_t*)pkt->data;
    m->tx_x_cm = (int16_t)(g_my_x_m * 100.0f);
    m->tx_y_cm = (int16_t)(g_my_y_m * 100.0f);
  } else if ((type == MSG_CLAIM) && (pkt->size == sizeof(msg_claim_t))) {
    msg_claim_t* m = (msg_claim_t*)pkt->data;
    m->tx_x_cm = (int16_t)(g_my_x_m * 100.0f);
    m->tx_y_cm = (int16_t)(g_my_y_m * 100.0f);
  } else if ((type == MSG_DONE) && (pkt->size == sizeof(msg_done_t))) {
    msg_done_t* m = (msg_done_t*)pkt->data;
    m->tx_x_cm = (int16_t)(g_my_x_m * 100.0f);
    m->tx_y_cm = (int16_t)(g_my_y_m * 100.0f);
  } else if ((type == MSG_SNAPSHOT_FR) &&
             (pkt->size == sizeof(msg_snapshot_frag_t))) {
    msg_snapshot_frag_t* m = (msg_snapshot_frag_t*)pkt->data;
    m->tx_x_cm = (int16_t)(g_my_x_m * 100.0f);
    m->tx_y_cm = (int16_t)(g_my_y_m * 100.0f);
  } else if ((type == MSG_DONE_LEDGER) &&
             (pkt->size == sizeof(msg_done_ledger_t))) {
    msg_done_ledger_t* m = (msg_done_ledger_t*)pkt->data;
    m->tx_x_cm = (int16_t)(g_my_x_m * 100.0f);
    m->tx_y_cm = (int16_t)(g_my_y_m * 100.0f);
  }
}
#endif

static void p2pRxCb(P2PPacket* p) {
  const uint8_t* b = (const uint8_t*)0;
  uint8_t type = 0u;
  uint8_t src = 0u;
  uint8_t tx = 0u;
  uint8_t seq = 0u;

  if ((p == (P2PPacket*)0) || (p->size < 6u)) {
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

#if USE_MESH
  if (b[4] > 1u) {
    g_relay_pkt.size = p->size;
    memcpy(g_relay_pkt.data, p->data, p->size);
    ((uint8_t*)g_relay_pkt.data)[2] = g_my_id;
    ((uint8_t*)g_relay_pkt.data)[4]--;
    ((uint8_t*)g_relay_pkt.data)[5]++;
    updateRelayTxPosition(type, &g_relay_pkt);
    radiolinkSendP2PPacketBroadcast(&g_relay_pkt);

    if (type != MSG_BEACON) {
      DEBUG_PRINT(
          "[RELAY] me=%s type=%u src=0x%02X tx=0x%02X seq=%u ttl=%u->%u "
          "hop=%u->%u\n",
          appNodeName(g_my_id), (unsigned)type, (unsigned)b[1],
          (unsigned)b[2], (unsigned)b[3], (unsigned)b[4],
          (unsigned)(((uint8_t*)g_relay_pkt.data)[4]), (unsigned)b[5],
          (unsigned)(((uint8_t*)g_relay_pkt.data)[5]));
    }
  }
#endif

  g_rx_count++;
  g_last_rx_ms[appNodeIndexFromId(src)] =
      (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  if ((type == MSG_CLAIM) && (p->size == sizeof(msg_claim_t))) {
    app_rx_event_t e;
    e.type = MSG_CLAIM;
    memcpy(&e.u.claim, p->data, sizeof(msg_claim_t));
    rxQueuePush(&e);
    return;
  }

  if ((type == MSG_DONE) && (p->size == sizeof(msg_done_t))) {
    app_rx_event_t e;
    e.type = MSG_DONE;
    memcpy(&e.u.done, p->data, sizeof(msg_done_t));
    rxQueuePush(&e);
    return;
  }

  if ((type == MSG_SNAPSHOT_FR) && (p->size == sizeof(msg_snapshot_frag_t))) {
    app_rx_event_t e;
    e.type = MSG_SNAPSHOT_FR;
    memcpy(&e.u.snapf, p->data, sizeof(msg_snapshot_frag_t));
    rxQueuePush(&e);
    return;
  }

  if ((type == MSG_DONE_LEDGER) && (p->size == sizeof(msg_done_ledger_t))) {
    app_rx_event_t e;
    e.type = MSG_DONE_LEDGER;
    memcpy(&e.u.ledger, p->data, sizeof(msg_done_ledger_t));
    rxQueuePush(&e);
    return;
  }

  if ((type == MSG_BEACON) && (p->size == sizeof(msg_beacon_t))) {
    static CRTPPacket crtp_pkt;
    crtp_pkt.header = CRTP_HEADER(CRTP_PORT_P2P_PROXY, 0);
    crtp_pkt.size = sizeof(msg_beacon_t);
    memcpy(crtp_pkt.data, p->data, sizeof(msg_beacon_t));
    crtpSendPacket(&crtp_pkt);
    return;
  }
}

void p2pCommInit(uint8_t my_radio_id) {
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

  DEBUG_PRINT(
      "[P2P_INIT] me=%s mesh=%u beacon_range=%u beacon_radius=%.2f "
      "cbba_range=%u cbba_radius=%.2f ttl=%u\n",
      appNodeName(g_my_id), (unsigned)USE_MESH,
      (unsigned)USE_BEACON_RANGE_LIMIT, (double)BEACON_RADIUS_M,
      (unsigned)USE_CBBA_RANGE_LIMIT, (double)CBBA_COMM_RADIUS_M,
      (unsigned)TTL_MAX);
}

static void sendPacket(const void* buf, uint8_t sz) {
  g_txp.size = sz;
  memcpy(g_txp.data, buf, sz);
  radiolinkSendP2PPacketBroadcast(&g_txp);
}

void p2pCommSendBeacon(const msg_beacon_t* m) {
  if (m == (const msg_beacon_t*)0) {
    return;
  }
  sendPacket(m, (uint8_t)sizeof(*m));

  static CRTPPacket crtp_tx;
  crtp_tx.header = CRTP_HEADER(CRTP_PORT_P2P_PROXY, 1);
  crtp_tx.size = sizeof(msg_beacon_t);
  memcpy(crtp_tx.data, m, sizeof(msg_beacon_t));
  crtpSendPacket(&crtp_tx);
}

void p2pCommSendClaim(const msg_claim_t* m) {
  if (m == (const msg_claim_t*)0) {
    return;
  }
  sendPacket(m, (uint8_t)sizeof(*m));
}

void p2pCommSendDone(const msg_done_t* m) {
  if (m == (const msg_done_t*)0) {
    return;
  }
  sendPacket(m, (uint8_t)sizeof(*m));
}

void p2pCommSendSnapshotFrag(const msg_snapshot_frag_t* m) {
  if (m == (const msg_snapshot_frag_t*)0) {
    return;
  }
  sendPacket(m, (uint8_t)sizeof(*m));
}

void p2pCommSendDoneLedger(const msg_done_ledger_t* m) {
  if (m == (const msg_done_ledger_t*)0) {
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

uint32_t p2pCommGetRxCount(void) { return g_rx_count; }

uint32_t p2pCommGetDropCount(void) { return g_drop_count; }

void p2pCommSetLocalPos(float x_m, float y_m) {
  g_my_x_m = x_m;
  g_my_y_m = y_m;
}
