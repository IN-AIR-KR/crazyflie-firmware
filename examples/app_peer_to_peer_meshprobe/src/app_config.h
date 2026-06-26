#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define NODE_ID_D1 0xE6u
#define NODE_ID_D2 0xE7u
#define NODE_ID_D3 0xE8u

#define AGENT_COUNT 3u

/* ============================================================
 * Communication mode
 * ------------------------------------------------------------
 * Mesh test:
 *   USE_MESH=1u, TTL_MAX=2u
 * P2P test:
 *   USE_MESH=0u, TTL_MAX=1u
 *
 * Demo target:
 * - D1 and D2 are direct neighbors.
 * - D2 and D3 are direct neighbors.
 * - D1 and D3 are outside direct range.
 * - P2P: D1 cannot hear D3 at mission start, so both can approach TASK0.
 * - Mesh: D2 relays D3's row to D1, so D1 starts on TASK1 instead.
 * ============================================================ */
#define USE_MESH 0u
#define TTL_MAX 1u

#define USE_RANGE_LIMIT 1u
#define COMM_RADIUS_M 0.9f

#define USE_CBBA_RANGE_LIMIT USE_RANGE_LIMIT
#define USE_CLAIM_RANGE_LIMIT USE_CBBA_RANGE_LIMIT
#define CBBA_COMM_RADIUS_M COMM_RADIUS_M

#define CLAIM_IMMEDIATE_SUFFIX_RELEASE 1u

/* mission switch */
#define MISSION_AUTO_START 1u

/* CBBA size */
#define TASK_MAX 8u
#define TASK_COUNT_RUNTIME 3u
#define BUNDLE_LIMIT 1u

/* 3-task demo: each drone completes one task, then hovers. */
#define DEMO_AGENT_TASK_CAP_ENABLE 1u
#define DEMO_D1_TASK_CAP 1u
#define DEMO_D2_TASK_CAP 1u
#define DEMO_D3_TASK_CAP 1u

/* flight / state */
#define LOOP_HZ 50u
#define BEACON_TX_HZ 10u
#define PEER_TO_MS 5000u
#define PEER_LOSS_STREAK_MS 3000u
#define START_HOLD_MS 3000u
#define TAKEOFF_MS 1200u
#define TAKEOFF_Z_M 0.60f
#define LAND_VZ_MPS (-0.18f)

/* communication / queue */
#define SEEN_N 128u
#define RX_QUEUE_N 24u

/* CBBA communication periods */
#define CLAIM_TX_PERIOD_MS 100u
#define ROLLING_AUCTION_ENABLE 1u
#define BIDVEC_TX_PERIOD_MS 100u
#define BIDVEC_STALE_MS 1800u
#define ACTIVE_EXEC_RESERVATION_LOCK_ENABLE 1u
#define DONE_REPEAT_PERIOD_MS 150u

/* P2P/Mesh delivery behavior */
#define P2P_DIRECT_ONLY_REJECT_RELAYED 1u
#define P2P_PROXY_BEACON_PERIOD_MS 200u
#define MESH_RELAY_CACHE_ENABLE 1u
#define MESH_RELAY_CACHE_N 6u
#define MESH_RELAY_REPEAT_COUNT 1u
#define MESH_RELAY_REPEAT_PERIOD_MS 35u

/* reservation lease / dynamic bid */
#define CLAIM_BID_REFRESH_MS 100u
#define CLAIM_BID_REFRESH_DELTA_Q 20
#define CLAIM_STALE_MS 1500u
#define CLAIM_BURST_COUNT 8u
#define TASK_DISTANCE_BID_Q_PER_M 1000

/* Equal task utilities.
 * With identical values, the bid is purely value minus distance cost.
 */
#define TASK0_VALUE_Q 26000
#define TASK1_VALUE_Q 26000
#define TASK2_VALUE_Q 26000

/* done */
#define DONE_RADIUS_M 0.28f
#define DONE_DWELL_MS 250u
#define LANE_ENTRY_RADIUS_M 0.28f

/* near-goal position hold */
#define GOAL_HOLD_RADIUS_M 0.25f

/* generic stabilization pause after losing an active auction target */
#define REPLAN_HOLD_MS 0u

/* no immediate task chase after takeoff */
#define POST_TAKEOFF_HOLD_MS 1800u

/* auction-only window after takeoff hold: comms run, XY still holds */
#define POST_TAKEOFF_AUCTION_WARMUP_MS 600u

/* XY velocity control */
#define XY_KP 0.8f
#define XY_VEL_MAX 0.19f

/* short-range peer collision avoidance */
#define APF_AVOID_ENABLE 1u
#define APF_RADIUS_M 0.20f
#define APF_HARD_RADIUS_M 0.12f
#define APF_VEL_MAX 0.05f
#define APF_PEER_POS_TIMEOUT_MS 2600u

/* final DONE spreading hold */
#define MISSION_DONE_HOLD_MS 1500u

/* ============================================================
 * 3-task hidden-terminal demo layout
 * ------------------------------------------------------------
 * Coordinate convention:
 *   +x : forward
 *   +y : left
 *
 * Expected behavior:
 *   Mesh ON : D3(E8)->TASK0, D2(E7)->TASK2,
 *             D1(E6)->TASK1 from the start.
 *   P2P    : D1(E6) and D3(E8) initially approach TASK0.
 *             D1 switches to TASK1 after direct communication or DONE.
 * ============================================================ */

/* TASK0/T1: D1/D3 duplicate target in P2P. */
#define TASK0_X_M (0.418f)
#define TASK0_Y_M (0.880f)
#define TASK0_END_X_M TASK0_X_M
#define TASK0_END_Y_M TASK0_Y_M

/* TASK1/T2: D1 fallback after D3 wins TASK0/T1. */
#define TASK1_X_M (-1.705f)
#define TASK1_Y_M (1.485f)
#define TASK1_END_X_M TASK1_X_M
#define TASK1_END_Y_M TASK1_Y_M

/* TASK2/T3: D2's lower task. */
#define TASK2_X_M (0.000f)
#define TASK2_Y_M (-0.759f)
#define TASK2_END_X_M TASK2_X_M
#define TASK2_END_Y_M TASK2_Y_M

/* D1-D2 and D2-D3 are inside COMM_RADIUS_M; D1-D3 is outside. */
#define D1_X0_M (-0.80f)
#define D1_Y0_M (0.00f)

#define D2_X0_M (0.00f)
#define D2_Y0_M (0.00f)

#define D3_X0_M (0.80f)
#define D3_Y0_M (0.00f)

#endif
