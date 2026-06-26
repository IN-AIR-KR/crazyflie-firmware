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
static float g_peer_x_m[AGENT_COUNT];
static float g_peer_y_m[AGENT_COUNT];
static uint8_t g_peer_pos_valid[AGENT_COUNT];
static uint16_t g_peer_total_dist_cm[AGENT_COUNT];
static uint8_t g_peer_done_count[AGENT_COUNT];
static uint8_t g_peer_claim_loss_count[AGENT_COUNT];
static uint8_t g_peer_app_state[AGENT_COUNT];
static uint8_t g_peer_start_ready[AGENT_COUNT];
static uint32_t g_proxy_last_ms[AGENT_COUNT];
static uint32_t g_rx_count = 0u;
static uint32_t g_drop_count = 0u;

static float g_my_x_m = 0.0f;
static float g_my_y_m = 0.0f;

static P2PPacket g_txp;
#if USE_MESH
static P2PPacket g_relay_pkt;
#if MESH_RELAY_CACHE_ENABLE
typedef struct {
  uint8_t valid;
  uint8_t type;
  uint8_t src_id;
  uint8_t seq;
  uint8_t repeat_left;
  uint32_t next_tx_ms;
  P2PPacket pkt;
} relay_cache_t;

static relay_cache_t g_relay_cache[MESH_RELAY_CACHE_N];
static uint8_t g_relay_cache_wr = 0u;
#endif
#endif

static bool proxyBeaconAllowed(uint8_t src_id) {
#if P2P_PROXY_BEACON_PERIOD_MS > 0u
  const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  uint8_t idx = 0u;

  if (!appIsValidNodeId(src_id)) {
    return false;
  }

  idx = appNodeIndexFromId(src_id);

  if ((g_proxy_last_ms[idx] != 0u) &&
      ((now_ms - g_proxy_last_ms[idx]) < P2P_PROXY_BEACON_PERIOD_MS)) {
    return false;
  }

  g_proxy_last_ms[idx] = now_ms;
#else
  (void)src_id;
#endif

  return true;
}

static void proxyRawToClient(const void* payload, uint8_t size, uint8_t channel) {
  static CRTPPacket crtp_tx;

  if ((payload == (const void*)0) || (size > (uint8_t)CRTP_MAX_DATA_SIZE)) {
    return;
  }

  crtp_tx.header = CRTP_HEADER(CRTP_PORT_P2P_PROXY, channel);
  crtp_tx.size = size;
  memcpy(crtp_tx.data, payload, size);
  crtpSendPacket(&crtp_tx);
}

static void proxyPacketToClient(const P2PPacket* p, uint8_t channel) {
  if (p == (const P2PPacket*)0) {
    return;
  }

  proxyRawToClient(p->data, p->size, channel);
}

static void rxQueueDropAtOffset(uint8_t offset) {
  uint8_t i = 0u;

  if (offset >= g_rx_count_q) {
    return;
  }

  for (i = offset; i + 1u < g_rx_count_q; i++) {
    const uint8_t dst = (uint8_t)((g_rx_head + i) % RX_QUEUE_N);
    const uint8_t src = (uint8_t)((g_rx_head + i + 1u) % RX_QUEUE_N);
    g_rx_q[dst] = g_rx_q[src];
  }

  if (g_rx_tail == 0u) {
    g_rx_tail = RX_QUEUE_N - 1u;
  } else {
    g_rx_tail--;
  }

  g_rx_count_q--;
}

static bool rxQueueMakeRoomForDone(void) {
  uint8_t i = 0u;

  if (g_rx_count_q < RX_QUEUE_N) {
    return true;
  }

  for (i = 0u; i < g_rx_count_q; i++) {
    const uint8_t idx = (uint8_t)((g_rx_head + i) % RX_QUEUE_N);

    if (g_rx_q[idx].type != MSG_DONE) {
      rxQueueDropAtOffset(i);
      g_drop_count++;
      return true;
    }
  }

  return false;
}

static bool seenHas(uint8_t src, uint8_t type, uint8_t seq) {
  uint8_t i = 0u;

  for (i = 0u; i < SEEN_N; i++) {
    if ((g_seen[i].src_id == src) && (g_seen[i].type == type) && (g_seen[i].seq == seq)) {
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
    if ((e != (const app_rx_event_t*)0) && (e->type == MSG_DONE) && rxQueueMakeRoomForDone()) {
      /* Space was made by dropping an older non-DONE event. */
    } else {
      g_drop_count++;
      return;
    }
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

#if USE_RANGE_LIMIT
static bool isWithinRadiusCm(int16_t tx_x_cm, int16_t tx_y_cm, float radius_m) {
  const float tx_x_m = (float)tx_x_cm * 0.01f;
  const float tx_y_m = (float)tx_y_cm * 0.01f;
  const float dx = tx_x_m - g_my_x_m;
  const float dy = tx_y_m - g_my_y_m;
  const float r2 = radius_m * radius_m;

  return ((dx * dx) + (dy * dy)) <= r2;
}
#endif

static bool packetPassesRangeGate(uint8_t type, const P2PPacket* p) {
  const uint8_t* b = (const uint8_t*)p->data;

#if P2P_DIRECT_ONLY_REJECT_RELAYED
#if !USE_MESH
  if ((type == MSG_CLAIM) || (type == MSG_BIDVEC)) {
    const uint8_t src_id = b[1];
    const uint8_t tx_id = b[2];
    const uint8_t hop = b[5];

    if ((tx_id != src_id) || (hop != 0u)) {
      return false;
    }
  }
#endif
#endif

#if USE_RANGE_LIMIT
  if (type == MSG_BEACON) {
    return true;
  }

  if (type == MSG_CLAIM) {
#if USE_CLAIM_RANGE_LIMIT
    if (p->size == sizeof(msg_claim_t)) {
      const msg_claim_t* m = (const msg_claim_t*)p->data;
      return isWithinRadiusCm(m->tx_x_cm, m->tx_y_cm, CBBA_COMM_RADIUS_M);
    }
#endif
    return true;
  }

  if (type == MSG_DONE) {
    return true;
  }

  if (type == MSG_BIDVEC) {
#if USE_CBBA_RANGE_LIMIT
    if (p->size == sizeof(msg_bidvec_t)) {
      const msg_bidvec_t* m = (const msg_bidvec_t*)p->data;
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
  } else if ((type == MSG_BIDVEC) && (pkt->size == sizeof(msg_bidvec_t))) {
    msg_bidvec_t* m = (msg_bidvec_t*)pkt->data;
    m->tx_x_cm = (int16_t)(g_my_x_m * 100.0f);
    m->tx_y_cm = (int16_t)(g_my_y_m * 100.0f);
  }
}

#if MESH_RELAY_CACHE_ENABLE
static uint8_t relayCacheTypeAllowed(uint8_t type) {
  return ((type == MSG_BIDVEC) || (type == MSG_CLAIM)) ? 1u : 0u;
}

static void relayCacheStore(uint8_t type, const P2PPacket* pkt) {
  const uint8_t* b = (const uint8_t*)0;
  uint8_t i = 0u;
  uint8_t slot = g_relay_cache_wr;
  const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  if ((relayCacheTypeAllowed(type) == 0u) || (pkt == (const P2PPacket*)0) || (pkt->size < 6u) ||
      (MESH_RELAY_REPEAT_COUNT == 0u)) {
    return;
  }

  b = (const uint8_t*)pkt->data;

  for (i = 0u; i < MESH_RELAY_CACHE_N; i++) {
    if ((g_relay_cache[i].valid != 0u) && (g_relay_cache[i].type == type) &&
        (g_relay_cache[i].src_id == b[1]) && (g_relay_cache[i].seq == b[3])) {
      slot = i;
      break;
    }
  }

  g_relay_cache[slot].valid = 1u;
  g_relay_cache[slot].type = type;
  g_relay_cache[slot].src_id = b[1];
  g_relay_cache[slot].seq = b[3];
  g_relay_cache[slot].repeat_left = MESH_RELAY_REPEAT_COUNT;
  g_relay_cache[slot].next_tx_ms = now_ms + MESH_RELAY_REPEAT_PERIOD_MS;
  g_relay_cache[slot].pkt.size = pkt->size;
  memcpy(g_relay_cache[slot].pkt.data, pkt->data, pkt->size);

  if (i >= MESH_RELAY_CACHE_N) {
    g_relay_cache_wr++;
    if (g_relay_cache_wr >= MESH_RELAY_CACHE_N) {
      g_relay_cache_wr = 0u;
    }
  }
}
#endif
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
  if ((b[4] > 1u) && (type != MSG_BEACON)) {
    g_relay_pkt.size = p->size;
    memcpy(g_relay_pkt.data, p->data, p->size);
    ((uint8_t*)g_relay_pkt.data)[2] = g_my_id;
    ((uint8_t*)g_relay_pkt.data)[4]--;
    ((uint8_t*)g_relay_pkt.data)[5]++;
    updateRelayTxPosition(type, &g_relay_pkt);
    radiolinkSendP2PPacketBroadcast(&g_relay_pkt);
#if MESH_RELAY_CACHE_ENABLE
    relayCacheStore(type, &g_relay_pkt);
#endif
  }
#endif

  g_rx_count++;
  g_last_rx_ms[appNodeIndexFromId(src)] = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  if ((type == MSG_BEACON) && (p->size == sizeof(msg_beacon_t))) {
    const msg_beacon_t* m = (const msg_beacon_t*)p->data;
    const uint8_t idx = appNodeIndexFromId(src);

    g_peer_x_m[idx] = (float)m->x_cm * 0.01f;
    g_peer_y_m[idx] = (float)m->y_cm * 0.01f;
    g_peer_total_dist_cm[idx] = m->total_dist_cm;
    g_peer_done_count[idx] = m->done_count;
    g_peer_claim_loss_count[idx] = m->claim_loss_count;
    g_peer_app_state[idx] = m->app_state;
    g_peer_start_ready[idx] = m->start_ready;
    g_peer_pos_valid[idx] = 1u;
  }

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
    proxyPacketToClient(p, 0u);
    rxQueuePush(&e);
    return;
  }

  if ((type == MSG_BIDVEC) && (p->size == sizeof(msg_bidvec_t))) {
    app_rx_event_t e;
    e.type = MSG_BIDVEC;
    memcpy(&e.u.bidv, p->data, sizeof(msg_bidvec_t));
    rxQueuePush(&e);
    return;
  }

  if ((type == MSG_BEACON) && (p->size == sizeof(msg_beacon_t))) {
    if (!proxyBeaconAllowed(src)) {
      return;
    }

    proxyPacketToClient(p, 0u);
    return;
  }
}

void p2pCommInit(uint8_t my_radio_id) {
  memset(g_seen, 0, sizeof(g_seen));
  memset(g_rx_q, 0, sizeof(g_rx_q));
  memset(g_last_rx_ms, 0, sizeof(g_last_rx_ms));
  memset(g_peer_x_m, 0, sizeof(g_peer_x_m));
  memset(g_peer_y_m, 0, sizeof(g_peer_y_m));
  memset(g_peer_pos_valid, 0, sizeof(g_peer_pos_valid));
  memset(g_peer_total_dist_cm, 0, sizeof(g_peer_total_dist_cm));
  memset(g_peer_done_count, 0, sizeof(g_peer_done_count));
  memset(g_peer_claim_loss_count, 0, sizeof(g_peer_claim_loss_count));
  memset(g_peer_app_state, 0, sizeof(g_peer_app_state));
  memset(g_peer_start_ready, 0, sizeof(g_peer_start_ready));
  memset(g_proxy_last_ms, 0, sizeof(g_proxy_last_ms));
#if USE_MESH
#if MESH_RELAY_CACHE_ENABLE
  memset(g_relay_cache, 0, sizeof(g_relay_cache));
#endif
#endif

  g_my_id = my_radio_id;
  g_seen_wr = 0u;
#if USE_MESH
#if MESH_RELAY_CACHE_ENABLE
  g_relay_cache_wr = 0u;
#endif
#endif
  g_rx_head = 0u;
  g_rx_tail = 0u;
  g_rx_count_q = 0u;
  g_rx_count = 0u;
  g_drop_count = 0u;

  p2pRegisterCB(p2pRxCb);
}

void p2pCommTick(void) {
#if USE_MESH
#if MESH_RELAY_CACHE_ENABLE
  const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  uint8_t i = 0u;

  for (i = 0u; i < MESH_RELAY_CACHE_N; i++) {
    relay_cache_t* r = &g_relay_cache[i];

    if ((r->valid == 0u) || (r->repeat_left == 0u)) {
      continue;
    }

    if (((int32_t)(now_ms - r->next_tx_ms)) < 0) {
      continue;
    }

    updateRelayTxPosition(r->type, &r->pkt);
    radiolinkSendP2PPacketBroadcast(&r->pkt);

    r->repeat_left--;
    if (r->repeat_left == 0u) {
      r->valid = 0u;
    } else {
      r->next_tx_ms = now_ms + MESH_RELAY_REPEAT_PERIOD_MS;
    }
  }
#endif
#endif
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

  if (!proxyBeaconAllowed(m->src_id)) {
    return;
  }

  proxyRawToClient(m, (uint8_t)sizeof(*m), 1u);
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
  proxyRawToClient(m, (uint8_t)sizeof(*m), 1u);
}

void p2pCommSendBidVec(const msg_bidvec_t* m) {
  if (m == (const msg_bidvec_t*)0) {
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

uint32_t p2pCommGetRxCount(void) {
  return g_rx_count;
}

uint32_t p2pCommGetDropCount(void) {
  return g_drop_count;
}

void p2pCommSetLocalPos(float x_m, float y_m) {
  g_my_x_m = x_m;
  g_my_y_m = y_m;
}

bool p2pCommGetPeerPos(uint8_t peer_radio_id, float* x_m, float* y_m, uint32_t* age_ms) {
  const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  uint8_t idx = 0u;

  if (!appIsValidNodeId(peer_radio_id) || (x_m == (float*)0) || (y_m == (float*)0)) {
    return false;
  }

  idx = appNodeIndexFromId(peer_radio_id);

  if ((g_peer_pos_valid[idx] == 0u) || (g_last_rx_ms[idx] == 0u)) {
    return false;
  }

  *x_m = g_peer_x_m[idx];
  *y_m = g_peer_y_m[idx];

  if (age_ms != (uint32_t*)0) {
    *age_ms = now_ms - g_last_rx_ms[idx];
  }

  return true;
}

bool p2pCommGetPeerMetric(uint8_t peer_radio_id, uint16_t* total_dist_cm, uint8_t* done_count,
                          uint8_t* claim_loss_count, uint32_t* age_ms) {
  const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  uint8_t idx = 0u;

  if (!appIsValidNodeId(peer_radio_id)) {
    return false;
  }

  idx = appNodeIndexFromId(peer_radio_id);

  if ((g_peer_pos_valid[idx] == 0u) || (g_last_rx_ms[idx] == 0u)) {
    return false;
  }

  if (total_dist_cm != (uint16_t*)0) {
    *total_dist_cm = g_peer_total_dist_cm[idx];
  }

  if (done_count != (uint8_t*)0) {
    *done_count = g_peer_done_count[idx];
  }

  if (claim_loss_count != (uint8_t*)0) {
    *claim_loss_count = g_peer_claim_loss_count[idx];
  }

  if (age_ms != (uint32_t*)0) {
    *age_ms = now_ms - g_last_rx_ms[idx];
  }

  return true;
}

bool p2pCommGetPeerStartInfo(uint8_t peer_radio_id, uint8_t* app_state, uint8_t* start_ready,
                             uint32_t* age_ms) {
  const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  uint8_t idx = 0u;

  if (!appIsValidNodeId(peer_radio_id)) {
    return false;
  }

  idx = appNodeIndexFromId(peer_radio_id);

  if ((g_peer_pos_valid[idx] == 0u) || (g_last_rx_ms[idx] == 0u)) {
    return false;
  }

  if (app_state != (uint8_t*)0) {
    *app_state = g_peer_app_state[idx];
  }

  if (start_ready != (uint8_t*)0) {
    *start_ready = g_peer_start_ready[idx];
  }

  if (age_ms != (uint32_t*)0) {
    *age_ms = now_ms - g_last_rx_ms[idx];
  }

  return true;
}
