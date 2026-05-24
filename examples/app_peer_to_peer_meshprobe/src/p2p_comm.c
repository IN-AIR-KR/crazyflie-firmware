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

/* ── 중복 수신 방지 (seen 캐시) ─────────────────────────────────────────── */
typedef struct {
  uint8_t src_id;
  uint8_t type;
  uint8_t seq;
} seen_t;

static uint8_t g_my_id = 0u;
static seen_t g_seen[SEEN_N];
static uint8_t g_seen_wr = 0u;

/* ── CBBA 이벤트 큐 ─────────────────────────────────────────────────────── */
static msg_cbba_state_t g_cbba_q[RX_QUEUE_N];
static uint8_t g_cbba_head = 0u;
static uint8_t g_cbba_tail = 0u;
static uint8_t g_cbba_count = 0u;

/* ── 통계 ────────────────────────────────────────────────────────────────── */
static uint32_t g_last_rx_ms[AGENT_COUNT];
static uint32_t g_rx_count = 0u;

/* ── 내 위치 (CBBA 패킷 거리 필터용) ────────────────────────────────────── */
static float g_my_x_m = 0.0f;
static float g_my_y_m = 0.0f;

/* ── peer 위치 캐시 (수신된 beacon 으로 갱신, CBBA 거리 체크에 사용) ──── */
static float g_peer_x_m[AGENT_COUNT];
static float g_peer_y_m[AGENT_COUNT];
static uint8_t g_peer_pos_valid[AGENT_COUNT];

/* ── 송신 버퍼 ──────────────────────────────────────────────────────────── */
static P2PPacket g_txp;

/* ── seen 캐시 ──────────────────────────────────────────────────────────── */
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

/* ── CBBA 이벤트 큐 push ─────────────────────────────────────────────────── */
static void cbbaQueuePush(const msg_cbba_state_t* m) {
  if (g_cbba_count >= RX_QUEUE_N) {
    return; /* 큐 가득 찼으면 드랍 */
  }

  g_cbba_q[g_cbba_tail] = *m;
  g_cbba_tail++;

  if (g_cbba_tail >= RX_QUEUE_N) {
    g_cbba_tail = 0u;
  }

  g_cbba_count++;
}

/* ── P2P 수신 콜백 ──────────────────────────────────────────────────────── */
static void p2pRxCb(P2PPacket* p) {
  const uint8_t* b = (const uint8_t*)0;
  uint8_t type = 0u;
  uint8_t src = 0u;
  uint8_t seq = 0u;
  uint8_t src_idx = 0u;

  if ((p == (P2PPacket*)0) || (p->size < 3u)) {
    return;
  }

  b = (const uint8_t*)p->data;
  type = b[0];
  src = b[1];
  seq = b[2];

  if (src == g_my_id) {
    return;
  }

  if (!appIsValidNodeId(src)) {
    return;
  }

  if (seenHas(src, type, seq)) {
    return;
  }

  seenPut(src, type, seq);

  src_idx = appNodeIndexFromId(src);

  /* ── MSG_BEACON: peer 위치 갱신 + CRTP ch0 전달 ─────────────────────── */
  if ((type == MSG_BEACON) && (p->size == sizeof(msg_beacon_t))) {
    const msg_beacon_t* bm = (const msg_beacon_t*)p->data;

    g_peer_x_m[src_idx] = (float)bm->x_cm * 0.01f;
    g_peer_y_m[src_idx] = (float)bm->y_cm * 0.01f;
    g_peer_pos_valid[src_idx] = 1u;

    g_rx_count++;
    g_last_rx_ms[src_idx] = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    {
      static CRTPPacket crtp_pkt;
      crtp_pkt.header = CRTP_HEADER(CRTP_PORT_P2P_PROXY, 0);
      crtp_pkt.size = sizeof(msg_beacon_t);
      memcpy(crtp_pkt.data, p->data, sizeof(msg_beacon_t));
      crtpSendPacket(&crtp_pkt);
    }

    return;
  }

  /* ── MSG_CBBA_STATE: 거리 필터 → 이벤트 큐 ─────────────────────────── */
  if ((type == MSG_CBBA_STATE) && (p->size == sizeof(msg_cbba_state_t))) {
#if USE_CBBA_RANGE_LIMIT
    if (g_peer_pos_valid[src_idx]) {
      const float dx = g_peer_x_m[src_idx] - g_my_x_m;
      const float dy = g_peer_y_m[src_idx] - g_my_y_m;

      if ((dx * dx + dy * dy) > (CBBA_COMM_RADIUS_M * CBBA_COMM_RADIUS_M)) {
        return;
      }
    }
#endif

    g_rx_count++;
    g_last_rx_ms[src_idx] = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    cbbaQueuePush((const msg_cbba_state_t*)p->data);
    return;
  }
}

/* ── 공개 API ─────────────────────────────────────────────────────────────── */

void p2pCommInit(uint8_t my_radio_id) {
  memset(g_seen, 0, sizeof(g_seen));
  memset(g_cbba_q, 0, sizeof(g_cbba_q));
  memset(g_last_rx_ms, 0, sizeof(g_last_rx_ms));
  memset(g_peer_x_m, 0, sizeof(g_peer_x_m));
  memset(g_peer_y_m, 0, sizeof(g_peer_y_m));
  memset(g_peer_pos_valid, 0, sizeof(g_peer_pos_valid));

  g_my_id = my_radio_id;
  g_seen_wr = 0u;
  g_cbba_head = 0u;
  g_cbba_tail = 0u;
  g_cbba_count = 0u;
  g_rx_count = 0u;

  p2pRegisterCB(p2pRxCb);

  DEBUG_PRINT("[P2P_INIT] me=0x%02X (%s)\n", (unsigned)my_radio_id, appNodeName(my_radio_id));
}

void p2pCommSendBeacon(const msg_beacon_t* m) {
  if (m == (const msg_beacon_t*)0) {
    return;
  }

  g_txp.size = sizeof(msg_beacon_t);
  memcpy(g_txp.data, m, sizeof(msg_beacon_t));
  radiolinkSendP2PPacketBroadcast(&g_txp);

  {
    static CRTPPacket crtp_tx;
    crtp_tx.header = CRTP_HEADER(CRTP_PORT_P2P_PROXY, 1);
    crtp_tx.size = sizeof(msg_beacon_t);
    memcpy(crtp_tx.data, m, sizeof(msg_beacon_t));
    crtpSendPacket(&crtp_tx);
  }
}

void p2pCommSendCbbaState(const msg_cbba_state_t* m) {
  if (m == (const msg_cbba_state_t*)0) {
    return;
  }

  g_txp.size = sizeof(msg_cbba_state_t);
  memcpy(g_txp.data, m, sizeof(msg_cbba_state_t));
  radiolinkSendP2PPacketBroadcast(&g_txp);
}

bool p2pCommPollCbbaState(msg_cbba_state_t* out) {
  if ((out == (msg_cbba_state_t*)0) || (g_cbba_count == 0u)) {
    return false;
  }

  *out = g_cbba_q[g_cbba_head];
  g_cbba_head++;

  if (g_cbba_head >= RX_QUEUE_N) {
    g_cbba_head = 0u;
  }

  g_cbba_count--;
  return true;
}

void p2pCommSetLocalPos(float x_m, float y_m) {
  g_my_x_m = x_m;
  g_my_y_m = y_m;
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
