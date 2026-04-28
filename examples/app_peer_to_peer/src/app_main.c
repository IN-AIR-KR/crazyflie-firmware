/*
* app_main.c (지금코드.odt 기반 + 리더(ROLE_LEADER)만 Push demo 이식)
*
* 요구사항:
* - 리더로 선정된 기체만 “push”: 손/벽이 가까우면 밀려나고,
* 한 번 밀린 후에는 원점으로 복귀하지 않고 그 위치를 새로운 hover 기준점으로 유지.
*
* 구현 요점:
* - Multiranger 4방향으로 repulsive 속도(vx_rep, vy_rep) 계산
* - 그 속도를 기준점(refX,refY)에 적분하여 ref 자체를 이동 (=> 원점 복귀 없음)
* - 리더는 modeAbs로 sp.position.x/y = refX/refY를 계속 넣어 hover 유지
* - 리더 전환 시 refValid=false로 재설정(Trigger/Decision 모두)
*
* 빌드 안정:
* - stateEstimateGetState() 미사용 (환경에 따라 미정의)
* - stabilizerGetState()만 사용
* - -Werror 기준 unused variable/function 제거/사용 처리
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#include "stabilizer_types.h"
#include "stabilizer.h"
#include "commander.h"

#include "range.h"
#include "led.h"

#define DEBUG_MODULE "QRELAY"
#include "debug.h"

/* ================= Timing ================= */
#define LOOP_HZ 50
#define MESH_TX_HZ 20
#define PEER_TO_MS 5000
#define PEER_LOSS_STREAK_MS 3000

#define START_HOLD_MS 3000
#define TAKEOFF_MS 1200

#define TAKEOFF_Z 0.60f
#define LAND_VZ (-0.18f)

/* ============== Leader trigger (front range) ============== */
#define FRONT_TRIG_M 0.15f
#define FRONT_DEB_N 4

/* ============== Leader hand-follow (4-way) ============== */
#define LEADER_TARGET_M 0.25f
#define LEADER_DB_M 0.03f
#define LEADER_KP 2.0f
#define LEADER_V_MAX 0.30f

/* ============== Leader PUSH demo (leader-only) ============== */
/* “밀리면 그 자리 유지”를 위해 ref 자체를 이동시키는 방식 */
#define PUSH_R0_M 0.60f /* 이 안으로 들어오면 밀림 */
#define PUSH_RMIN_M 0.08f /* 1/d 폭주 방지 하한 */
#define PUSH_K 0.30f /* repulsion gain */
#define PUSH_V_MAX 0.35f /* repulsive vel clamp */
#define PUSH_REF_LIMIT_M 6.0f /* ref drift safety clamp */

/* leader ref state */
static bool leaderRefValid = false;
static float leaderRefX = 0.0f;
static float leaderRefY = 0.0f;

/* ============== Relay LED ============== */
static void ledRelayOn(void)
{
ledInit();
ledSet(LED_RED_L, true);
ledSet(LED_RED_R, true);
ledSet(LED_BLUE_L, false);
ledSet(LED_BLUE_NRF, false);
}
static void ledRelayOff(void)
{
ledInit();
ledSet(LED_RED_L, false);
ledSet(LED_RED_R, false);
ledSet(LED_BLUE_L, true);
ledSet(LED_BLUE_NRF, true);
}

/* ============== Mesh message types ============== */
#define MSG_BEACON 2u
#define MSG_ROLE 3u
#define MSG_BID 4u
#define MSG_DECISION 5u
#define MSG_POS 6u
#define MSG_POS_RPT 7u

#define MESH_TTL_INIT 2u
#define SEEN_N 64u

#define DEC_BURST_MS 3000
#define DEC_RETX_MS 2000
#define DEC_TX_PERIOD_MS 200
#define DEC_KEEPALIVE_MS 1000

/* ============== Debug log throttling ============== */
#define HB_LOG_MS 1000u
#define LINK_LOG_MS 500u
#define PATH_LOG_MS 500u
#define RELAY_OUT_LOG_MS 250u

/* ============== Link-quality estimation ============== */
#define Q_EMA_ALPHA 0.20f

/* direct/RSSI 중심, hop/age 영향 축소 */
#define W_DIRECT 1.20f
#define W_RSSI 1.00f
#define W_HOP 0.25f
#define W_AGE 0.15f

/* ===== Election objective ===== */
#define LAMBDA_BAL 0.60f
#define MU_DQL 0.40f
#define NU_HOP 0.35f
#define BID_EPS_S16 80

/* ===== Position compare (optional relay report) ===== */
#define POS_TX_HZ 10u
#define POS_SRC_ID 0xE8u
#define POS_MM_SCALE 1000.0f

/* ================= RSSI normalization (unsaturated) ================= */
#define RSSI_RAW_MIN 40.0f
#define RSSI_RAW_MAX 100.0f

/* ================= J metric scaling ================= */
#define J_BAL_W 0.50f
#define J_GAIN 3.0f

/* ================= Relay motion / smoothing ================= */
#define RELAY_DIR_N 16u

#define RELAY_PROBE_MOVE_MS 50u
#define RELAY_PROBE_HOLD_MS 50u
#define RELAY_CURR_HOLD_MS 70u

#define RELAY_STEP_V 0.19f
#define RELAY_STEP_MOVE_MS 80u
#define RELAY_SETTLE_MS 40u

#define RELAY_CREEP_V 0.07f

#define RELAY_V_MAX 0.52f
#define RELAY_V_LPF_ALPHA 0.34f

#define RELAY_A_MAX 0.50f
#define RELAY_DV_MAX (RELAY_A_MAX / (float)LOOP_HZ)

#define DIR_ALPHA 0.22f
#define SOFTMAX_BETA 2.3f
#define RELAY_STEP_V_MIN 0.05f
#define RELAY_STEP_V_MAX 0.20f
#define RELAY_BAL_GAIN 0.70f
#define RELAY_BAL_CAP 0.90f
#define RELAY_BAL_HOLD_ERR 0.06f
#define RELAY_BAL_HOLD_WEAK 1.80f
#define RELAY_SCORE_DBAL 14.0f
#define RELAY_SCORE_DWEAK 2.0f
#define RELAY_SCORE_RESBAL 12.0f
#define RELAY_SCORE_DEAD 1.5f
#define RELAY_PROBE_V 0.14f
#define RELAY_STEP_ABORT_ERR 14.0f
#define RELAY_STEP_ABORT_MARGIN 4.0f
#define RELAY_Q_FAST_ALPHA 0.68f
#define RELAY_Q_SLOW_ALPHA 0.10f
#define RELAY_SCORE_SLOW_ALPHA 0.10f
#define RELAY_DSCORE_GAIN 1.80f
#define RELAY_DSCORE_PROBE_GAIN 1.20f
#define RELAY_WEAK_TARGET 1.92f
#define RELAY_WEAK_DEF_GAIN 0.55f
#define RELAY_DSCORE_DEAD 0.02f

/* ================= Model prior (probe ordering only) ================= */
#define MODEL_A0_DBM (-45.0f)
#define MODEL_N (2.2f)
#define MODEL_MIN_D_M (0.20f)
#define MODEL_MAX_D_M (20.0f)
#define MODEL_PROBE_PRED_STEP_M (0.22f * ((float)RELAY_PROBE_MOVE_MS / 1000.0f))

/* ================= Utils ================= */
static inline float clampf(const float x, const float lo, const float hi)
{
return (x < lo) ? lo : ((x > hi) ? hi : x);
}
static inline float absf(const float x)
{
return (x >= 0.0f) ? x : (-x);
}
static inline bool isValidId(const uint8_t id)
{
return (id == 0xE6u) || (id == 0xE7u) || (id == 0xE8u);
}
static inline float lpf1(const float prev, const float x, const float a)
{
return (prev + (a * (x - prev)));
}
static inline float slew1(const float prev, const float target, const float dv_max)
{
const float dv = target - prev;
if (dv > dv_max) { return (prev + dv_max); }
if (dv < -dv_max) { return (prev - dv_max); }
return target;
}
static inline int16_t clamp_s16(const int32_t v)
{
if (v > 32767) { return 32767; }
if (v < -32768) { return -32768; }
return (int16_t)v;
}
static inline uint16_t u16_now_ms(const TickType_t nowT)
{
const uint32_t ms = (uint32_t)(nowT * portTICK_PERIOD_MS);
return (uint16_t)(ms & 0xFFFFu);
}

/* ---------- Range helpers (NO enum types in signature) ---------- */
static bool rangeValToMetersValid(float d_raw, float *out_m)
{
if (d_raw <= 0.0f) { return false; }

/* 일부 드라이버에서 mm로 들어오는 경우 보정 */
if (d_raw > 2.0f) { d_raw *= 0.001f; }

if ((d_raw < 0.05f) || (d_raw > 2.00f)) { return false; }
*out_m = d_raw;
return true;
}

static bool rangeFrontValid(float *out_m) { return rangeValToMetersValid(rangeGet(rangeFront), out_m); }
static bool rangeBackValid (float *out_m) { return rangeValToMetersValid(rangeGet(rangeBack ), out_m); }
static bool rangeLeftValid (float *out_m) { return rangeValToMetersValid(rangeGet(rangeLeft ), out_m); }
static bool rangeRightValid(float *out_m) { return rangeValToMetersValid(rangeGet(rangeRight), out_m); }

/* 리더 트리거는 기존처럼 front만 */
static bool frontTrigValid(float *out_m) { return rangeFrontValid(out_m); }



/* ================= Leader PUSH demo helpers ================= */
static void leaderPushComputeRepulsiveVel(float *vx_rep, float *vy_rep)
{
float vx = 0.0f;
float vy = 0.0f;

/* Front: 가까우면 뒤로 (-x) */
{
float d = 0.0f;
if (rangeFrontValid(&d)) {
const float dd = clampf(d, PUSH_RMIN_M, 2.0f);
if (dd < PUSH_R0_M) {
const float g = PUSH_K * ((1.0f / dd) - (1.0f / PUSH_R0_M));
vx -= g;
}
}
}

/* Back: 가까우면 앞으로 (+x) */
{
float d = 0.0f;
if (rangeBackValid(&d)) {
const float dd = clampf(d, PUSH_RMIN_M, 2.0f);
if (dd < PUSH_R0_M) {
const float g = PUSH_K * ((1.0f / dd) - (1.0f / PUSH_R0_M));
vx += g;
}
}
}

/* Left: 가까우면 오른쪽 (-y) */
{
float d = 0.0f;
if (rangeLeftValid(&d)) {
const float dd = clampf(d, PUSH_RMIN_M, 2.0f);
if (dd < PUSH_R0_M) {
const float g = PUSH_K * ((1.0f / dd) - (1.0f / PUSH_R0_M));
vy -= g;
}
}
}

/* Right: 가까우면 왼쪽 (+y) */
{
float d = 0.0f;
if (rangeRightValid(&d)) {
const float dd = clampf(d, PUSH_RMIN_M, 2.0f);
if (dd < PUSH_R0_M) {
const float g = PUSH_K * ((1.0f / dd) - (1.0f / PUSH_R0_M));
vy += g;
}
}
}

*vx_rep = clampf(vx, -PUSH_V_MAX, PUSH_V_MAX);
*vy_rep = clampf(vy, -PUSH_V_MAX, PUSH_V_MAX);
}

/* 핵심: repulsive velocity를 ref에 적분 => 원점 복귀 없음 */
static void leaderPushUpdateRef(const float curX, const float curY,
const float vx_rep, const float vy_rep)
{
const float dt = (1.0f / (float)LOOP_HZ);

if (!leaderRefValid) {
leaderRefX = curX;
leaderRefY = curY;
leaderRefValid = true;
}

leaderRefX += (vx_rep * dt);
leaderRefY += (vy_rep * dt);

leaderRefX = clampf(leaderRefX, -PUSH_REF_LIMIT_M, PUSH_REF_LIMIT_M);
leaderRefY = clampf(leaderRefY, -PUSH_REF_LIMIT_M, PUSH_REF_LIMIT_M);
}

/* ================= Packets ================= */
typedef struct __attribute__((packed)) {
uint8_t type;
uint8_t src_id;
uint8_t tx_id;
uint8_t seq;
uint8_t ttl;
uint8_t hop;
uint16_t t_ms;
} mesh_beacon_t;

typedef struct __attribute__((packed)) {
uint8_t type;
uint8_t src_id;
uint8_t tx_id;
uint8_t seq;
uint8_t ttl;
uint8_t hop;
uint16_t t_ms;
} mesh_role_t;

typedef struct __attribute__((packed)) {
uint8_t type;
uint8_t src_id;
uint8_t tx_id;
uint8_t seq;
uint8_t ttl;
uint8_t hop;
int16_t bid_s16;
} mesh_bid_t;

typedef struct __attribute__((packed)) {
uint8_t type;
uint8_t src_id;
uint8_t tx_id;
uint8_t seq;
uint8_t ttl;
uint8_t hop;
uint8_t leader_id;
uint8_t relay_id;
uint8_t source_id;
} mesh_decision_t;

typedef struct __attribute__((packed)) {
uint8_t type;
uint8_t src_id;
uint8_t tx_id;
uint8_t seq;
uint8_t ttl;
uint8_t hop;
uint16_t t_ms;
int16_t x_mm;
int16_t y_mm;
int16_t z_mm;
} mesh_pos_t;

typedef struct __attribute__((packed)) {
uint8_t type;
uint8_t src_id;
uint8_t tx_id;
uint8_t seq;
uint8_t ttl;
uint8_t hop;
uint16_t t_ms;
int8_t rssi_in;
int16_t x_mm;
int16_t y_mm;
int16_t z_mm;
} mesh_pos_rpt_t;

/* ================= Dedup ================= */
typedef struct { uint8_t src_id; uint8_t seq; } seen_t;
static seen_t seen_buf[SEEN_N];
static uint8_t seen_wr = 0u;

static bool seenHas(const uint8_t src_id, const uint8_t seq)
{
uint8_t i;
for (i = 0u; i < (uint8_t)SEEN_N; i++) {
if ((seen_buf[i].src_id == src_id) && (seen_buf[i].seq == seq)) { return true; }
}
return false;
}
static void seenPut(const uint8_t src_id, const uint8_t seq)
{
seen_buf[seen_wr].src_id = src_id;
seen_buf[seen_wr].seq = seq;
seen_wr++;
if (seen_wr >= (uint8_t)SEEN_N) { seen_wr = 0u; }
}

/* ================= Peer state ================= */
typedef struct {
bool alive;
int rssi_last;
TickType_t last_rx_tick;

uint8_t last_tx;
uint8_t last_hop;
uint8_t last_ttl;

float q_ema;

bool has_bid;
int16_t bid_s16;
} peer_t;

static peer_t pE6, pE7, pE8;

static peer_t* peerById(const uint8_t id)
{
if (id == 0xE6u) { return &pE6; }
if (id == 0xE7u) { return &pE7; }
if (id == 0xE8u) { return &pE8; }
return (peer_t*)0;
}
static bool peerAlive(const peer_t *p, const TickType_t nowT)
{
if (p == (peer_t*)0) { return false; }
return p->alive && ((nowT - p->last_rx_tick) < M2T(PEER_TO_MS));
}

/* ================= Positions for model prior ================= */
typedef struct {
bool valid;
float x;
float y;
float z;
TickType_t last_tick;
} node_pos_t;

static node_pos_t posE6;
static node_pos_t posE8;

static void posUpdate(node_pos_t *p, const int16_t x_mm, const int16_t y_mm, const int16_t z_mm, const TickType_t nowT)
{
if (p == (node_pos_t*)0) { return; }
p->valid = true;
p->x = ((float)x_mm) / POS_MM_SCALE;
p->y = ((float)y_mm) / POS_MM_SCALE;
p->z = ((float)z_mm) / POS_MM_SCALE;
p->last_tick = nowT;
}

/* ================= Roles ================= */
typedef enum { ROLE_NONE=0, ROLE_LEADER, ROLE_RELAY, ROLE_SOURCE } role_t;
static uint8_t my_id = 0u;
static role_t my_role = ROLE_NONE;

static uint8_t leader_id = 0u;
static uint8_t relay_id = 0u;
static uint8_t source_id = 0u;

static bool decision_set = false;

static TickType_t role_lock_t0 = 0;
static TickType_t decision_burst_until = 0;
static TickType_t decision_retx_until = 0;
static uint8_t g_leader_role_seq = 0u;

/* ================= seq + packet ================= */
static uint8_t seq_beacon = 0u;
static uint8_t seq_role = 0u;
static uint8_t seq_bid = 0u;
static uint8_t seq_dec = 0u;
static uint8_t seq_pos = 0u;

static P2PPacket txp;

/* ================= Debug-only path logs ================= */
typedef struct {
bool direct_valid;
int direct_rssi_raw;
uint8_t direct_hop;
uint8_t direct_tx;
float direct_q_inst;
TickType_t direct_tick;

bool via_valid;
int via_lr_rssi_raw;
int via_rs_rssi_raw;
uint8_t via_hop;
uint8_t via_tx;
float via_q_lr_inst;
float via_q_rs_inst;
float via_q_path;
TickType_t via_tick;
} source_path_log_t;

static source_path_log_t g_src_path_log;

/* ================= Mesh forward helper ================= */
static void meshForwardIfTtl(const void *rx, const uint32_t sz, const uint8_t ttl, const uint8_t hop)
{
if (ttl == 0u) { return; }
if (sz > 32u) { return; }

uint8_t buf[32];
memcpy(buf, rx, sz);
buf[2] = my_id;
buf[4] = (uint8_t)(ttl - 1u);
buf[5] = (uint8_t)(hop + 1u);

txp.size = (uint8_t)sz;
memcpy(txp.data, buf, sz);
radiolinkSendP2PPacketBroadcast(&txp);
}

/* ================= Link quality ================= */
static float qualityInstant(const int rssi_raw_i, const uint8_t hop, const bool is_direct, const uint32_t age_ms)
{
const float direct = is_direct ? 1.0f : 0.0f;

const float rssi_raw = (float)rssi_raw_i;
const float denom = (RSSI_RAW_MAX - RSSI_RAW_MIN);
float rssi_good = 0.0f;
if (denom > 1.0e-6f) {
rssi_good = (RSSI_RAW_MAX - rssi_raw) / denom;
}
rssi_good = clampf(rssi_good, 0.0f, 1.0f);

const float hop_f = (float)hop;
const float age_norm = clampf(((float)age_ms) / (float)PEER_TO_MS, 0.0f, 1.0f);

float q = 0.0f;
q += (W_DIRECT * direct);
q += (W_RSSI * rssi_good);
q -= (W_HOP * hop_f);
q -= (W_AGE * age_norm);
return q;
}

/* ================= Election ================= */
static float g_prev_qL_elect = 0.0f;

static int16_t computeBidQBalanced(const TickType_t nowT, const uint8_t leader, const uint8_t other_nonleader)
{
const peer_t *pL = peerById(leader);
const peer_t *pO = peerById(other_nonleader);

float qL = 0.0f;
float qO = 0.0f;
float hopL = 2.0f;
float hopO = 2.0f;

if ((pL != (peer_t*)0) && peerAlive(pL, nowT)) { qL = pL->q_ema; hopL = (float)pL->last_hop; }
if ((pO != (peer_t*)0) && peerAlive(pO, nowT)) { qO = pO->q_ema; hopO = (float)pO->last_hop; }

{
const float weak = (qL < qO) ? qL : qO;
const float diff = absf(qL - qO);
const float dql = absf(qL - g_prev_qL_elect);
g_prev_qL_elect = qL;

const float score = weak - (LAMBDA_BAL * diff) - (MU_DQL * dql) - (NU_HOP * (hopL + hopO));
const float sc = clampf(score, -30.0f, 30.0f);
return (int16_t)lrintf(sc * 1000.0f);
}
}

static uint8_t tieBreakId(const uint8_t a, const uint8_t b)
{
if ((g_leader_role_seq & 1u) != 0u) { return (a > b) ? a : b; }
return (a < b) ? a : b;
}

/* ================= Directions ================= */
typedef enum
{
DIR_0 = 0,
DIR_1,
DIR_2,
DIR_3,
DIR_4,
DIR_5,
DIR_6,
DIR_7,
DIR_8,
DIR_9,
DIR_10,
DIR_11,
DIR_12,
DIR_13,
DIR_14,
DIR_15
} dir_t;

static void __attribute__((unused)) dirVec(const dir_t d, float * const vx, float * const vy)
{
static const float ux_tbl[RELAY_DIR_N] = {
1.00000000f, 0.92387953f, 0.70710678f, 0.38268343f,
0.00000000f, -0.38268343f, -0.70710678f, -0.92387953f,
-1.00000000f, -0.92387953f, -0.70710678f, -0.38268343f,
0.00000000f, 0.38268343f, 0.70710678f, 0.92387953f
};
static const float uy_tbl[RELAY_DIR_N] = {
0.00000000f, 0.38268343f, 0.70710678f, 0.92387953f,
1.00000000f, 0.92387953f, 0.70710678f, 0.38268343f,
0.00000000f, -0.38268343f, -0.70710678f, -0.92387953f,
-1.00000000f, -0.92387953f, -0.70710678f, -0.38268343f
};
const uint8_t idx = (uint8_t)d;
if (idx < (uint8_t)RELAY_DIR_N) {
*vx = ux_tbl[idx];
*vy = uy_tbl[idx];
} else {
*vx = 1.0f;
*vy = 0.0f;
}
}

/* ================= Relay metric ================= */
static void relayGetQLQS(const TickType_t nowT, float *qL, float *qS)
{
const peer_t *pL = peerById(leader_id);
const peer_t *pS = peerById(source_id);

float L = 0.0f;
float S = 0.0f;

if ((pL != (peer_t*)0) && peerAlive(pL, nowT)) {
const float ageL = clampf(((float)((nowT - pL->last_rx_tick) * portTICK_PERIOD_MS)) / (float)PEER_TO_MS, 0.0f, 1.0f);
L = pL->q_ema - (0.35f * ageL);
}

if ((pS != (peer_t*)0) && peerAlive(pS, nowT)) {
const float ageS = clampf(((float)((nowT - pS->last_rx_tick) * portTICK_PERIOD_MS)) / (float)PEER_TO_MS, 0.0f, 1.0f);
S = pS->q_ema - (0.35f * ageS);
}

*qL = L;
*qS = S;
}

static float relayJFromQLQS(const float qL, const float qS)
{
const float weak = (qL < qS) ? qL : qS;
const float bal = absf(qL - qS);
return ((3.0f * weak) - (3.8f * bal));
}

/* ================= Probe order ================= */
static void __attribute__((unused)) buildProbeOrder(uint8_t order[RELAY_DIR_N], const float myx, const float myy)
{
uint8_t i;
(void)myx;
(void)myy;
for (i = 0u; i < (uint8_t)RELAY_DIR_N; i++) {
order[i] = i;
}
}

/* ================= RX callback ================= */
static void rxCb(P2PPacket *p)
{
if ((p == (P2PPacket*)0) || (p->size < 6u)) { return; }

const uint8_t *b = (const uint8_t*)p->data;
const uint8_t type = b[0];
const uint8_t src_id = b[1];
const uint8_t tx_id = b[2];
const uint8_t seq = b[3];
const uint8_t ttl = b[4];
const uint8_t hop = b[5];

if (src_id == my_id) { return; }
if (!isValidId(src_id)) { return; }

/* Dedup (POS_RPT 분리) */
{
uint8_t dedup_src = src_id;
if (type == MSG_POS_RPT) { dedup_src = (uint8_t)(src_id ^ 0x80u); }
if (seenHas(dedup_src, seq)) { return; }
seenPut(dedup_src, seq);
}

/* peer 갱신 */
{
peer_t *pp = peerById(src_id);
if (pp != (peer_t*)0) {
const TickType_t nowT = xTaskGetTickCount();
const uint32_t age_ms = (uint32_t)((nowT - pp->last_rx_tick) * portTICK_PERIOD_MS);

pp->alive = true;
pp->rssi_last = (int)p->rssi;
pp->last_rx_tick = nowT;

pp->last_tx = tx_id;
pp->last_hop = hop;
pp->last_ttl = ttl;

{
const bool is_direct = ((tx_id == src_id) && (hop == 0u));
const float q_i = qualityInstant((int)p->rssi, hop, is_direct, age_ms);
if (pp->q_ema == 0.0f) { pp->q_ema = q_i; }
else { pp->q_ema = lpf1(pp->q_ema, q_i, Q_EMA_ALPHA); }
}
}
}

if ((type == MSG_ROLE) && (p->size == sizeof(mesh_role_t))) {
const mesh_role_t *rx = (const mesh_role_t*)p->data;
g_leader_role_seq = rx->seq;

if ((role_lock_t0 == 0) && (my_role == ROLE_NONE)) {
leader_id = rx->src_id;
role_lock_t0 = xTaskGetTickCount();
}

meshForwardIfTtl(rx, sizeof(mesh_role_t), rx->ttl, rx->hop);
return;
}

if ((type == MSG_BID) && (p->size == sizeof(mesh_bid_t))) {
const mesh_bid_t *rx = (const mesh_bid_t*)p->data;

peer_t *pp = peerById(rx->src_id);
if (pp != (peer_t*)0) { pp->has_bid = true; pp->bid_s16 = rx->bid_s16; }

meshForwardIfTtl(rx, sizeof(mesh_bid_t), rx->ttl, rx->hop);
return;
}

if ((type == MSG_DECISION) && (p->size == sizeof(mesh_decision_t))) {
const mesh_decision_t *rx = (const mesh_decision_t*)p->data;

if (!decision_set) {
leader_id = rx->leader_id;
relay_id = rx->relay_id;
source_id = rx->source_id;
decision_set = true;

if (my_id == leader_id) {
my_role = ROLE_LEADER;
decision_retx_until = xTaskGetTickCount() + M2T(DEC_RETX_MS);

/* PUSH DEMO: leader 전환 시 ref 재설정 */
leaderRefValid = false;
} else if (my_id == relay_id) {
my_role = ROLE_RELAY;
ledRelayOn();
} else if (my_id == source_id) {
my_role = ROLE_SOURCE;
} else {
my_role = ROLE_NONE;
}
}

meshForwardIfTtl(rx, sizeof(mesh_decision_t), rx->ttl, rx->hop);
return;
}

if ((type == MSG_POS) && (p->size == sizeof(mesh_pos_t))) {
const mesh_pos_t *rx = (const mesh_pos_t*)p->data;
const TickType_t nowT = xTaskGetTickCount();

if (rx->src_id == 0xE6u) { posUpdate(&posE6, rx->x_mm, rx->y_mm, rx->z_mm, nowT); }
if (rx->src_id == 0xE8u) { posUpdate(&posE8, rx->x_mm, rx->y_mm, rx->z_mm, nowT); }

if (decision_set && (my_role == ROLE_SOURCE) && (rx->src_id == leader_id) && (rx->tx_id == leader_id)) {
const float q_dir = qualityInstant((int)p->rssi, rx->hop, true, 0u);
g_src_path_log.direct_valid = true;
g_src_path_log.direct_rssi_raw = (int)p->rssi;
g_src_path_log.direct_hop = rx->hop;
g_src_path_log.direct_tx = rx->tx_id;
g_src_path_log.direct_q_inst = q_dir;
g_src_path_log.direct_tick = nowT;
}

/* 릴레이가 E8의 POS를 받을 때만 RPT를 쏨 */
if (decision_set && (my_role == ROLE_RELAY) && (rx->src_id == (uint8_t)POS_SRC_ID)) {
mesh_pos_rpt_t rp;
memset(&rp, 0, sizeof(rp));
rp.type = MSG_POS_RPT;
rp.src_id = rx->src_id;
rp.tx_id = my_id;
rp.seq = rx->seq;
rp.ttl = 0u;
rp.hop = 0u;
rp.t_ms = u16_now_ms(nowT);
rp.rssi_in = (int8_t)p->rssi;
rp.x_mm = rx->x_mm;
rp.y_mm = rx->y_mm;
rp.z_mm = rx->z_mm;

txp.size = (uint8_t)sizeof(mesh_pos_rpt_t);
memcpy(txp.data, &rp, sizeof(mesh_pos_rpt_t));
radiolinkSendP2PPacketBroadcast(&txp);
}
return;
}

if ((type == MSG_POS_RPT) && (p->size == sizeof(mesh_pos_rpt_t))) {
const mesh_pos_rpt_t *rx = (const mesh_pos_rpt_t*)p->data;
const TickType_t nowT = xTaskGetTickCount();

if (decision_set && (my_role == ROLE_SOURCE) && (rx->src_id == leader_id) && (rx->tx_id == relay_id)) {
const float q_lr = qualityInstant((int)rx->rssi_in, 0u, true, 0u);
const float q_rs = qualityInstant((int)p->rssi, rx->hop, true, 0u);
const float q_path = (q_lr < q_rs) ? q_lr : q_rs;

g_src_path_log.via_valid = true;
g_src_path_log.via_lr_rssi_raw = (int)rx->rssi_in;
g_src_path_log.via_rs_rssi_raw = (int)p->rssi;
g_src_path_log.via_hop = rx->hop;
g_src_path_log.via_tx = rx->tx_id;
g_src_path_log.via_q_lr_inst = q_lr;
g_src_path_log.via_q_rs_inst = q_rs;
g_src_path_log.via_q_path = q_path;
g_src_path_log.via_tick = nowT;
}
return;
}

if ((type == MSG_BEACON) && (p->size == sizeof(mesh_beacon_t))) {
const mesh_beacon_t *rx = (const mesh_beacon_t*)p->data;
meshForwardIfTtl(rx, sizeof(mesh_beacon_t), rx->ttl, rx->hop);
return;
}
}

/* ================= Main ================= */
typedef enum { ST_IDLE=0, ST_TAKEOFF, ST_RUN, ST_LAND, ST_OFF } st_t;

typedef enum
{
RP_MEAS_CURR = 0,
RP_PROBE_MOVE,
RP_PROBE_HOLD,
RP_STEP_MOVE,
RP_STEP_SETTLE
} relay_plan_state_t;

void appMain(void)
{
vTaskDelay(M2T(4000));

my_id = (uint8_t)(configblockGetRadioAddress() & 0xFFu);

memset(seen_buf, 0, sizeof(seen_buf));
seen_wr = 0u;

memset(&pE6, 0, sizeof(peer_t));
memset(&pE7, 0, sizeof(peer_t));
memset(&pE8, 0, sizeof(peer_t));

memset(&posE6, 0, sizeof(posE6));
memset(&posE8, 0, sizeof(posE8));
memset(&g_src_path_log, 0, sizeof(g_src_path_log));

p2pRegisterCB(rxCb);

memset(&txp, 0, sizeof(txp));
txp.port = 0x00;

DEBUG_PRINT("QRELAY: START my_id=0x%02X valid=%u\n", (unsigned)my_id, (unsigned)isValidId(my_id));

st_t st = ST_IDLE;
TickType_t st_t0 = xTaskGetTickCount();
TickType_t stable_t0 = 0;

TickType_t lastWake = xTaskGetTickCount();
TickType_t lastBeaconTx = 0;
TickType_t lastRoleTx = 0;
TickType_t lastBidTx = 0;
TickType_t lastDecTx = 0;
TickType_t lastPosTx = 0;

TickType_t lastHB = 0;
TickType_t lastLinkLog = 0;
TickType_t lastPathLog = 0;
TickType_t peer_loss_t0 = 0;

uint8_t front_deb = 0u;

/* Relay planner vars */
relay_plan_state_t rp = RP_MEAS_CURR;
TickType_t rp_t0 = 0;

float J_sum = 0.0f;
uint16_t J_n = 0u;
float qL_sum = 0.0f;
float qS_sum = 0.0f;
float bal_ref = 0.0f;
float weak_ref = 0.0f;

float score_dir[RELAY_DIR_N] __attribute__((unused));
uint8_t probe_order[RELAY_DIR_N] __attribute__((unused));
uint8_t probe_k = 0u;
dir_t probe_dir = DIR_0;

float target_ux = 1.0f;
float target_uy = 0.0f;

float last_ux = 1.0f;
float last_uy = 0.0f;

float dir_x_f = 1.0f;
float dir_y_f = 0.0f;

float vx_des = 0.0f;
float vy_des = 0.0f;

float vx_out = 0.0f;
float vy_out = 0.0f;
TickType_t relay_dbg_last = 0; // 200ms throttle
uint8_t relay_dbg_best_dir = 0; // 마지막 best_dir 저장
float relay_dbg_best_score = -999.0f;

while (1) {
vTaskDelayUntil(&lastWake, M2T(1000 / LOOP_HZ));
const TickType_t nowT = xTaskGetTickCount();

const bool aE6 = peerAlive(&pE6, nowT);
const bool aE7 = peerAlive(&pE7, nowT);
const bool aE8 = peerAlive(&pE8, nowT);

bool other_two_alive = false;
if (my_id == 0xE6u) { other_two_alive = (aE7 && aE8); }
if (my_id == 0xE7u) { other_two_alive = (aE6 && aE8); }
if (my_id == 0xE8u) { other_two_alive = (aE6 && aE7); }

state_t me;
stabilizerGetState(&me);

if ((nowT - lastHB) >= M2T(HB_LOG_MS)) {
lastHB = nowT;
DEBUG_PRINT("QRELAY: HB st=%d role=%d dec=%u L=%02X R=%02X S=%02X | rssi(E6,E7,E8)=%d,%d,%d\n",
(int)st, (int)my_role, (unsigned)decision_set,
(unsigned)leader_id, (unsigned)relay_id, (unsigned)source_id,
(int)pE6.rssi_last, (int)pE7.rssi_last, (int)pE8.rssi_last);
}

if ((my_role == ROLE_RELAY) && decision_set && ((nowT - lastLinkLog) >= M2T(LINK_LOG_MS))) {
float qL = 0.0f;
float qS = 0.0f;
lastLinkLog = nowT;
relayGetQLQS(nowT, &qL, &qS);

DEBUG_PRINT("QRELAY: LINK qL=%d qS=%d weak=%d bal=%d J=%d | L=%02X R=%02X S=%02X\n",
(int)(qL * 100.0f),
(int)(qS * 100.0f),
(int)(((qL < qS) ? qL : qS) * 100.0f),
(int)(absf(qL - qS) * 100.0f),
(int)(relayJFromQLQS(qL, qS) * 100.0f),
(unsigned)leader_id,
(unsigned)relay_id,
(unsigned)source_id);
}

if ((my_role == ROLE_SOURCE) && decision_set && ((nowT - lastPathLog) >= M2T(PATH_LOG_MS))) {
lastPathLog = nowT;
if (g_src_path_log.direct_valid || g_src_path_log.via_valid) {
DEBUG_PRINT("QRELAY: PATH dirQ=%d dirR=%d viaQ=%d lr=%d rs=%d | dirTx=%02X dirHop=%u viaTx=%02X viaHop=%u\n",
(int)(g_src_path_log.direct_q_inst * 100.0f),
(int)g_src_path_log.direct_rssi_raw,
(int)(g_src_path_log.via_q_path * 100.0f),
(int)g_src_path_log.via_lr_rssi_raw,
(int)g_src_path_log.via_rs_rssi_raw,
(unsigned)g_src_path_log.direct_tx,
(unsigned)g_src_path_log.direct_hop,
(unsigned)g_src_path_log.via_tx,
(unsigned)g_src_path_log.via_hop);
}
}

/* ================= FSM ================= */
switch (st) {
case ST_IDLE:
if (other_two_alive) {
if (stable_t0 == 0) { stable_t0 = nowT; }
if ((nowT - stable_t0) > M2T(START_HOLD_MS)) {
st = ST_TAKEOFF;
st_t0 = nowT;
DEBUG_PRINT("QRELAY: -> TAKEOFF\n");
}
} else {
stable_t0 = 0;
}
break;

case ST_TAKEOFF:
if ((nowT - st_t0) > M2T(TAKEOFF_MS)) {
st = ST_RUN;
st_t0 = nowT;
DEBUG_PRINT("QRELAY: -> RUN\n");
}
break;

case ST_RUN:
if (!other_two_alive) {
if (peer_loss_t0 == 0) { peer_loss_t0 = nowT; }
if ((nowT - peer_loss_t0) > M2T(PEER_LOSS_STREAK_MS)) {
st = ST_LAND;
st_t0 = nowT;
DEBUG_PRINT("QRELAY: peer lost(streak) -> LAND\n");
}
} else {
peer_loss_t0 = 0;
}
break;

case ST_LAND:
if (me.position.z < 0.08f) {
if (my_role == ROLE_RELAY) { ledRelayOff(); }
st = ST_OFF;
DEBUG_PRINT("QRELAY: -> OFF\n");
}
break;

default:
break;
}

/* ================= BEACON TX ================= */
if ((nowT - lastBeaconTx) >= M2T(1000 / MESH_TX_HZ)) {
lastBeaconTx = nowT;

mesh_beacon_t tx;
memset(&tx, 0, sizeof(tx));
tx.type = MSG_BEACON;
tx.src_id = my_id;
tx.tx_id = my_id;
tx.seq = seq_beacon++;
tx.ttl = (uint8_t)MESH_TTL_INIT;
tx.hop = 0u;
tx.t_ms = (uint16_t)(nowT & 0xFFFFu);

txp.size = (uint8_t)sizeof(mesh_beacon_t);
memcpy(txp.data, &tx, sizeof(mesh_beacon_t));
radiolinkSendP2PPacketBroadcast(&txp);
}

/* ================= POS TX (all) ================= */
if ((st == ST_RUN) && ((nowT - lastPosTx) >= M2T(1000 / POS_TX_HZ))) {
lastPosTx = nowT;

/* stabilizerGetState() 기반 */
state_t s;
stabilizerGetState(&s);

const int32_t x_mm = (int32_t)lrintf(s.position.x * POS_MM_SCALE);
const int32_t y_mm = (int32_t)lrintf(s.position.y * POS_MM_SCALE);
const int32_t z_mm = (int32_t)lrintf(s.position.z * POS_MM_SCALE);

mesh_pos_t px;
memset(&px, 0, sizeof(px));
px.type = MSG_POS;
px.src_id = my_id;
px.tx_id = my_id;
px.seq = seq_pos++;
px.ttl = 0u;
px.hop = 0u;
px.t_ms = u16_now_ms(nowT);
px.x_mm = clamp_s16(x_mm);
px.y_mm = clamp_s16(y_mm);
px.z_mm = clamp_s16(z_mm);

txp.size = (uint8_t)sizeof(mesh_pos_t);
memcpy(txp.data, &px, sizeof(mesh_pos_t));
radiolinkSendP2PPacketBroadcast(&txp);
}

/* ================= Leader trigger (front only) ================= */
if ((st == ST_RUN) && (my_role == ROLE_NONE) && (role_lock_t0 == 0)) {
float f = 0.0f;
const bool ok = frontTrigValid(&f);

if (ok && (f < FRONT_TRIG_M)) {
if (front_deb < 255u) { front_deb++; }
} else {
front_deb = 0u;
}

if (front_deb >= (uint8_t)FRONT_DEB_N) {
my_role = ROLE_LEADER;
leader_id = my_id;
role_lock_t0 = nowT;

decision_set = false;
relay_id = 0u;
source_id = 0u;
decision_burst_until = 0;
decision_retx_until = 0;

/* PUSH DEMO: leader 전환 시 ref 재설정 */
leaderRefValid = false;

DEBUG_PRINT("QRELAY: TRIG -> LEADER %02X\n", (unsigned)leader_id);
}
}

/* ================= Leader ROLE TX ================= */
if ((st == ST_RUN) && (my_role == ROLE_LEADER)) {
if ((nowT - lastRoleTx) >= M2T(200)) {
lastRoleTx = nowT;

mesh_role_t tx;
memset(&tx, 0, sizeof(tx));
tx.type = MSG_ROLE;
tx.src_id = my_id;
tx.tx_id = my_id;
tx.seq = seq_role++;
tx.ttl = (uint8_t)MESH_TTL_INIT;
tx.hop = 0u;
tx.t_ms = (uint16_t)(nowT & 0xFFFFu);

txp.size = (uint8_t)sizeof(mesh_role_t);
memcpy(txp.data, &tx, sizeof(mesh_role_t));
radiolinkSendP2PPacketBroadcast(&txp);
}
}

/* ================= Non-leader election ================= */
if ((st == ST_RUN) && (my_role != ROLE_LEADER) &&
(role_lock_t0 != 0) && (leader_id != 0u) && (!decision_set)) {

uint8_t other_id = 0u;
if (leader_id == 0xE6u) { other_id = (my_id == 0xE7u) ? 0xE8u : 0xE7u; }
if (leader_id == 0xE7u) { other_id = (my_id == 0xE6u) ? 0xE8u : 0xE6u; }
if (leader_id == 0xE8u) { other_id = (my_id == 0xE6u) ? 0xE7u : 0xE6u; }

if ((other_id != 0u) && (other_id != my_id) && (other_id != leader_id)) {

const int16_t my_bid = computeBidQBalanced(nowT, leader_id, other_id);

if ((nowT - lastBidTx) >= M2T(200)) {
lastBidTx = nowT;

mesh_bid_t tx;
memset(&tx, 0, sizeof(tx));
tx.type = MSG_BID;
tx.src_id = my_id;
tx.tx_id = my_id;
tx.seq = seq_bid++;
tx.ttl = (uint8_t)MESH_TTL_INIT;
tx.hop = 0u;
tx.bid_s16 = my_bid;

txp.size = (uint8_t)sizeof(mesh_bid_t);
memcpy(txp.data, &tx, sizeof(mesh_bid_t));
radiolinkSendP2PPacketBroadcast(&txp);
}

peer_t *pOther = peerById(other_id);
if ((pOther != (peer_t*)0) && pOther->has_bid) {
const int16_t ob = pOther->bid_s16;

uint8_t win = my_id;
const int32_t diff = (int32_t)ob - (int32_t)my_bid;
const int32_t adiff = (diff >= 0) ? diff : (-diff);

if (ob > my_bid) { win = other_id; }
else if (ob == my_bid) { win = tieBreakId(my_id, other_id); }
else { /* keep */ }

if (adiff < (int32_t)BID_EPS_S16) { win = tieBreakId(my_id, other_id); }

relay_id = win;
source_id = (relay_id == my_id) ? other_id : my_id;

decision_set = true;
if (my_id == relay_id) { my_role = ROLE_RELAY; ledRelayOn(); }
else { my_role = ROLE_SOURCE; }

decision_burst_until = nowT + M2T(DEC_BURST_MS);

DEBUG_PRINT("QRELAY: DECIDE(Q-bal): L=%02X R=%02X S=%02X | my_role=%d | myBid=%d otherBid=%d eps=%u seq=%u\n",
(unsigned)leader_id, (unsigned)relay_id, (unsigned)source_id,
(int)my_role, (int)my_bid, (int)ob, (unsigned)BID_EPS_S16, (unsigned)seq_bid);
}
}
}

/* ================= DECISION TX keepalive (+burst) ================= */
if (decision_set && (leader_id != 0u) && (relay_id != 0u) && (source_id != 0u)) {
const bool in_burst = (decision_burst_until != 0) && (nowT < decision_burst_until);
const TickType_t period = in_burst ? M2T(DEC_TX_PERIOD_MS) : M2T(DEC_KEEPALIVE_MS);

if ((nowT - lastDecTx) >= period) {
lastDecTx = nowT;

mesh_decision_t td;
memset(&td, 0, sizeof(td));
td.type = MSG_DECISION;
td.src_id = my_id;
td.tx_id = my_id;
td.seq = seq_dec++;
td.ttl = (uint8_t)MESH_TTL_INIT;
td.hop = 0u;
td.leader_id = leader_id;
td.relay_id = relay_id;
td.source_id = source_id;

txp.size = (uint8_t)sizeof(mesh_decision_t);
memcpy(txp.data, &td, sizeof(mesh_decision_t));
radiolinkSendP2PPacketBroadcast(&txp);
}
}

/* ================= RELAY movement (symmetric 4-dir hill-climb + equal q scaling) ================= */
vx_des = 0.0f;
vy_des = 0.0f;

if ((st == ST_RUN) && (my_role == ROLE_RELAY) && decision_set &&
    (leader_id != 0u) && (source_id != 0u)) {

  const float good_diff_enter = 0.03f;
  const float good_weak_enter = 1.74f;
  const float good_diff_exit = 0.08f;
  const float good_weak_exit = 1.68f;

  const float Q_RES_SCALE = 100.0f;
  const float SCORE_A = 2.0f;
  const float SCORE_B = 1.5f;
  const float SCORE_C = 1.2f;
  const float DROP_LIM = 8.0f;          /* scaled units: 0.08 */
  const float WEAK_DROP_LIM = 6.0f;     /* scaled units: 0.06 */
  const float DIFF_WORSE_LIM = 10.0f;   /* scaled units: 0.10 */
  const float HARD_REJECT_SCORE = -1000.0f;

  const float commit_v = 0.14f;
  const TickType_t commit_move_ms = M2T(RELAY_STEP_MOVE_MS);
  const TickType_t commit_settle_ms = M2T(RELAY_SETTLE_MS);

  float qL_now = 0.0f;
  float qS_now = 0.0f;
  float weak_now = 0.0f;
  float diff_now = 0.0f;
  static bool relay_good_hold = false;

  relayGetQLQS(nowT, &qL_now, &qS_now);
  weak_now = (qL_now < qS_now) ? qL_now : qS_now;
  diff_now = absf(qL_now - qS_now);

  if (rp_t0 == 0) {
    uint8_t i;
    for (i = 0u; i < (uint8_t)RELAY_DIR_N; i++) {
      score_dir[i] = HARD_REJECT_SCORE;
      probe_order[i] = i;
    }
    probe_order[0] = (uint8_t)DIR_0;   /* +x */
    probe_order[1] = (uint8_t)DIR_8;   /* -x */
    probe_order[2] = (uint8_t)DIR_4;   /* +y */
    probe_order[3] = (uint8_t)DIR_12;  /* -y */
    probe_k = 0u;
    probe_dir = DIR_0;
    rp = RP_MEAS_CURR;
    rp_t0 = nowT;

    target_ux = 1.0f;
    target_uy = 0.0f;
    last_ux = 1.0f;
    last_uy = 0.0f;
    dir_x_f = 1.0f;
    dir_y_f = 0.0f;

    J_sum = 0.0f;
    J_n = 0u;
    qL_sum = 0.0f;
    qS_sum = 0.0f;
    bal_ref = diff_now;
    weak_ref = weak_now;
    relay_dbg_best_dir = 0u;
    relay_dbg_best_score = HARD_REJECT_SCORE;
    relay_good_hold = false;
  }

  if ((qL_now < 0.50f) || (qS_now < 0.50f)) {
    rp = RP_MEAS_CURR;
    rp_t0 = nowT;
    relay_good_hold = false;
    vx_des = 0.0f;
    vy_des = 0.0f;
  }
  else if (rp == RP_MEAS_CURR) {
    vx_des = 0.0f;
    vy_des = 0.0f;
    relay_dbg_best_score = relayJFromQLQS(qL_now, qS_now);

    if (!relay_good_hold) {
      if ((diff_now <= good_diff_enter) && (weak_now >= good_weak_enter)) {
        relay_good_hold = true;
      }
    } else {
      if ((diff_now >= good_diff_exit) || (weak_now <= good_weak_exit)) {
        relay_good_hold = false;
      }
    }

    if (!relay_good_hold) {
      score_dir[(uint8_t)DIR_0] = HARD_REJECT_SCORE;
      score_dir[(uint8_t)DIR_8] = HARD_REJECT_SCORE;
      score_dir[(uint8_t)DIR_4] = HARD_REJECT_SCORE;
      score_dir[(uint8_t)DIR_12] = HARD_REJECT_SCORE;
      probe_k = 0u;
      probe_dir = (dir_t)probe_order[0];
      weak_ref = weak_now;
      bal_ref = diff_now;
      qL_sum = 0.0f;
      qS_sum = 0.0f;
      J_n = 0u;
      rp = RP_PROBE_MOVE;
      rp_t0 = nowT;
    }
  }
  else if (rp == RP_PROBE_MOVE) {
    float ux = 0.0f;
    float uy = 0.0f;
    dirVec(probe_dir, &ux, &uy);
    vx_des = ux * RELAY_PROBE_V;
    vy_des = uy * RELAY_PROBE_V;
    if ((nowT - rp_t0) >= M2T(RELAY_PROBE_MOVE_MS)) {
      qL_sum = 0.0f;
      qS_sum = 0.0f;
      J_n = 0u;
      rp = RP_PROBE_HOLD;
      rp_t0 = nowT;
    }
  }
  else if (rp == RP_PROBE_HOLD) {
    vx_des = 0.0f;
    vy_des = 0.0f;
    qL_sum += qL_now;
    qS_sum += qS_now;
    J_n++;

    if ((nowT - rp_t0) >= M2T(RELAY_PROBE_HOLD_MS)) {
      float avg_qL = qL_now;
      float avg_qS = qS_now;
      float avg_weak = 0.0f;
      float avg_diff = 0.0f;
      float qL_ref_s = qL_now * Q_RES_SCALE;
      float qS_ref_s = qS_now * Q_RES_SCALE;
      float avg_qL_s = 0.0f;
      float avg_qS_s = 0.0f;
      float weak_ref_s = 0.0f;
      float weak_now_s = 0.0f;
      float diff_ref_s = 0.0f;
      float diff_now_s = 0.0f;
      float dropL = 0.0f;
      float dropS = 0.0f;
      float dropSum = 0.0f;
      float probe_score = HARD_REJECT_SCORE;

      if (J_n > 0u) {
        avg_qL = qL_sum / (float)J_n;
        avg_qS = qS_sum / (float)J_n;
      }
      avg_weak = (avg_qL < avg_qS) ? avg_qL : avg_qS;
      avg_diff = absf(avg_qL - avg_qS);

      weak_ref_s = ((qL_ref_s < qS_ref_s) ? qL_ref_s : qS_ref_s);
      avg_qL_s = avg_qL * Q_RES_SCALE;
      avg_qS_s = avg_qS * Q_RES_SCALE;
      weak_now_s = ((avg_qL_s < avg_qS_s) ? avg_qL_s : avg_qS_s);
      diff_ref_s = absf(qL_ref_s - qS_ref_s);
      diff_now_s = absf(avg_qL_s - avg_qS_s);
      dropL = clampf(qL_ref_s - avg_qL_s, 0.0f, 1.0e9f);
      dropS = clampf(qS_ref_s - avg_qS_s, 0.0f, 1.0e9f);
      dropSum = dropL + dropS;

      probe_score = (SCORE_A * (weak_now_s - weak_ref_s))
                  + (SCORE_B * (diff_ref_s - diff_now_s))
                  - (SCORE_C * dropSum);

      if ((dropL > DROP_LIM) || (dropS > DROP_LIM) || ((weak_ref_s - weak_now_s) > WEAK_DROP_LIM)) {
        probe_score = HARD_REJECT_SCORE;
      }
      if ((diff_now_s - diff_ref_s) > DIFF_WORSE_LIM) {
        probe_score -= 200.0f;
      }

      score_dir[(uint8_t)probe_dir] = probe_score;
      probe_k++;
      qL_sum = 0.0f;
      qS_sum = 0.0f;
      J_n = 0u;

      if (probe_k >= 4u) {
        float best_s = score_dir[(uint8_t)DIR_0];
        uint8_t best_i = (uint8_t)DIR_0;
        float ux = 0.0f;
        float uy = 0.0f;

        if (score_dir[(uint8_t)DIR_8] > best_s) { best_s = score_dir[(uint8_t)DIR_8]; best_i = (uint8_t)DIR_8; }
        if (score_dir[(uint8_t)DIR_4] > best_s) { best_s = score_dir[(uint8_t)DIR_4]; best_i = (uint8_t)DIR_4; }
        if (score_dir[(uint8_t)DIR_12] > best_s) { best_s = score_dir[(uint8_t)DIR_12]; best_i = (uint8_t)DIR_12; }

        relay_dbg_best_dir = best_i;
        relay_dbg_best_score = best_s;

        if (best_s <= 0.0f) {
          relay_good_hold = ((diff_now <= good_diff_enter) && (weak_now >= good_weak_enter));
          rp = RP_MEAS_CURR;
          rp_t0 = nowT;
        } else {
          dirVec((dir_t)best_i, &ux, &uy);
          target_ux = ux;
          target_uy = uy;
          dir_x_f = target_ux;
          dir_y_f = target_uy;
          DEBUG_PRINT("QRELAY: RELAY_DEC best=%u score=%d tgt=%d,%d s=%d/%d/%d/%d\n",
                      (unsigned)best_i,
                      (int)(best_s),
                      (int)(target_ux * 1000.0f),
                      (int)(target_uy * 1000.0f),
                      (int)(score_dir[(uint8_t)DIR_0]),
                      (int)(score_dir[(uint8_t)DIR_8]),
                      (int)(score_dir[(uint8_t)DIR_4]),
                      (int)(score_dir[(uint8_t)DIR_12]));
          rp = RP_STEP_MOVE;
          rp_t0 = nowT;
        }
      } else {
        probe_dir = (dir_t)probe_order[probe_k];
        rp = RP_PROBE_MOVE;
        rp_t0 = nowT;
      }
    }
  }
  else if (rp == RP_STEP_MOVE) {
    dir_x_f = target_ux;
    dir_y_f = target_uy;
    vx_des = dir_x_f * commit_v;
    vy_des = dir_y_f * commit_v;
    if ((nowT - rp_t0) >= commit_move_ms) {
      rp = RP_STEP_SETTLE;
      rp_t0 = nowT;
    }
  }
  else if (rp == RP_STEP_SETTLE) {
    vx_des = 0.0f;
    vy_des = 0.0f;
    if ((nowT - rp_t0) >= commit_settle_ms) {
      rp = RP_MEAS_CURR;
      rp_t0 = nowT;
    }
  }

  {
    const float vx_lpf = lpf1(vx_out, vx_des, RELAY_V_LPF_ALPHA);
    const float vy_lpf = lpf1(vy_out, vy_des, RELAY_V_LPF_ALPHA);
    const float vx_slew = slew1(vx_out, vx_lpf, RELAY_DV_MAX);
    const float vy_slew = slew1(vy_out, vy_lpf, RELAY_DV_MAX);
    vx_out = clampf(vx_slew, -RELAY_V_MAX, RELAY_V_MAX);
    vy_out = clampf(vy_slew, -RELAY_V_MAX, RELAY_V_MAX);
  }

  if ((relay_dbg_last == 0) || ((nowT - relay_dbg_last) >= M2T(RELAY_OUT_LOG_MS))) {
    relay_dbg_last = nowT;
    DEBUG_PRINT("QRELAY: RELAY_PROBE rp=%u e=%d qL=%d qS=%d weak=%d diff=%d vx=%d vy=%d dir=%d,%d best=%u bs=%d\n",
                (unsigned)rp,
                (int)((qL_now - qS_now) * 100.0f),
                (int)(qL_now * 100.0f),
                (int)(qS_now * 100.0f),
                (int)(weak_now * 100.0f),
                (int)(diff_now * 100.0f),
                (int)(vx_out * 1000.0f),
                (int)(vy_out * 1000.0f),
                (int)(dir_x_f * 1000.0f),
                (int)(dir_y_f * 1000.0f),
                (unsigned)relay_dbg_best_dir,
                (int)(relay_dbg_best_score));
  }
} else {
  /* relay not active */
  rp_t0 = 0;
  vx_out = 0.0f;
  vy_out = 0.0f;
}

/* ================= Setpoint ================= */
{
setpoint_t sp;
memset(&sp, 0, sizeof(sp));

if (st == ST_LAND) {
sp.mode.z = modeVelocity;
sp.velocity.z = LAND_VZ;
commanderSetSetpoint(&sp, 3);
} else if ((st == ST_TAKEOFF) || (st == ST_RUN)) {
sp.mode.z = modeAbs;
sp.position.z = TAKEOFF_Z;

if ((st == ST_RUN) && (my_role == ROLE_LEADER)) {
/* ===== 리더만 PUSH DEMO ===== */
float vx_rep = 0.0f;
float vy_rep = 0.0f;

leaderPushComputeRepulsiveVel(&vx_rep, &vy_rep);
leaderPushUpdateRef(me.position.x, me.position.y, vx_rep, vy_rep);

sp.mode.x = modeAbs;
sp.mode.y = modeAbs;
sp.position.x = leaderRefX;
sp.position.y = leaderRefY;

commanderSetSetpoint(&sp, 3);
} else {
/* ===== 리더가 아니면 기존 velocity ===== */
sp.mode.x = modeVelocity;
sp.mode.y = modeVelocity;

float vx = 0.0f;
float vy = 0.0f;

if ((st == ST_RUN) && (my_role == ROLE_RELAY)) {
vx = vx_out;
vy = vy_out;
} else if ((st == ST_RUN) && (my_role == ROLE_NONE)) {
vx = 0.0f;
vy = 0.0f;
} else if ((st == ST_RUN) && (my_role == ROLE_SOURCE)) {
vx = 0.0f;
vy = 0.0f;
} else {
vx = 0.0f;
vy = 0.0f;
}

sp.velocity.x = vx;
sp.velocity.y = vy;
commanderSetSetpoint(&sp, 3);
}
}
}
}
}

/*
* 준수 요약:
* - 동적 메모리 할당 없음
* - 예외(try/catch) 없음
* - stateEstimateGetState() 미사용(빌드 환경 의존 제거)
* - -Werror: unused-function 방지(ledRelayOff 호출), unused 변수 제거
*/
