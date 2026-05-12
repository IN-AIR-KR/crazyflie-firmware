#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define NODE_ID_D1 0xE6u
#define NODE_ID_D2 0xE7u
#define NODE_ID_D3 0xE8u

#define AGENT_COUNT 3u

/* ============================================================
 * Communication mode
 * ------------------------------------------------------------
 * P2P test:
 *   #define USE_MESH 0u
 *   #define TTL_MAX 1u
 *
 * Mesh test:
 *   #define USE_MESH 1u
 *   #define TTL_MAX 2u
 *
 * BEACON range and CBBA range are intentionally separated.
 * - BEACON: alive / synchronization / visualization
 * - CBBA  : CLAIM / DONE / SNAPSHOT task-consensus traffic
 * ============================================================ */
#define USE_MESH 0u
#define TTL_MAX 1u

#define USE_RANGE_LIMIT 1u
#define USE_BEACON_RANGE_LIMIT 1u
#define USE_CBBA_RANGE_LIMIT 1u

#define BEACON_RADIUS_M 4.50f
#define CBBA_COMM_RADIUS_M 1.79f

/* mission switch */
#define MISSION_AUTO_START 1u
// 0 = mesh communication only, no takeoff
// 1 = auto takeoff + mission after startup synchronization

/* CBBA size */
#define TASK_MAX 9u
#define TASK_COUNT_RUNTIME 9u
#define BUNDLE_LIMIT 3u

/* snapshot fragment */
#define SNAP_FRAG_TASKS 3u
#define SNAP_FRAG_COUNT \
  ((TASK_COUNT_RUNTIME + SNAP_FRAG_TASKS - 1u) / SNAP_FRAG_TASKS)

/* flight / state */
#define LOOP_HZ 50u
#define BEACON_TX_HZ 20u
#define PEER_TO_MS 5000u
#define PEER_LOSS_STREAK_MS 8000u
#define START_HOLD_MS 3000u
#define TAKEOFF_MS 1200u
#define TAKEOFF_Z_M 0.60f
#define LAND_VZ_MPS (-0.18f)

/* communication / queue */
#define SEEN_N 128u
#define RX_QUEUE_N 32u

/* CBBA communication periods */
#define SNAPSHOT_TX_PERIOD_MS 1000u
#define CLAIM_TX_PERIOD_MS 75u
#define DONE_REPEAT_PERIOD_MS 75u
#define DONE_LEDGER_PERIOD_MS 75u
#define SUMMARY_LOG_MS 1000u
#define STATE_LOG_MS 1000u

/* task table/path/bundle diagnostic log */
#define TASK_TABLE_DEBUG_ENABLE 1u
#define TASK_TABLE_DEBUG_PERIOD_MS 2000u

/* done判定 */
#define DONE_RADIUS_M 0.28f
#define DONE_DWELL_MS 250u

/* near-goal position hold */
#define GOAL_HOLD_RADIUS_M 0.25f

/* no immediate task chase after takeoff */
#define POST_TAKEOFF_HOLD_MS 1800u

/* XY velocity control */
#define XY_KP 0.8f
#define XY_VEL_MAX 0.19f

/* continuous visit */
#define REPLAN_HOLD_MS 0u

/* final DONE spreading hold */
#define MISSION_DONE_HOLD_MS 3000u

/* ============================================================
 * 9-task layout for +/-2 m room
 * Coordinate convention:
 *   +x : forward
 *   +y : left
 * ============================================================ */
#define TASK0_X_M (-0.39f)
#define TASK0_Y_M (-0.45f)

#define TASK1_X_M ( 0.16f)
#define TASK1_Y_M ( 0.76f)

#define TASK2_X_M ( 1.31f)
#define TASK2_Y_M ( 0.53f)

#define TASK3_X_M ( 0.36f)
#define TASK3_Y_M (-0.97f)

#define TASK4_X_M (-1.80f)
#define TASK4_Y_M ( 0.16f)

#define TASK5_X_M (-1.57f)
#define TASK5_Y_M (-0.50f)

#define TASK6_X_M (-0.48f)
#define TASK6_Y_M ( 1.60f)

#define TASK7_X_M ( 1.60f)
#define TASK7_Y_M (-0.22f)

#define TASK8_X_M (-1.22f)
#define TASK8_Y_M (-1.60f)

/* ============================================================
 * Initial physical layout
 * With CBBA_COMM_RADIUS_M=1.79:
 *   D1-D2 ~= 1.54 m < 1.79  -> direct CBBA neighbor
 *   D2-D3 ~= 1.56 m < 1.79  -> direct CBBA neighbor
 *   D1-D3 ~= 2.86 m > 1.79  -> not direct in P2P
 * ============================================================ */
#define D1_X0_M (-1.02f)
#define D1_Y0_M (-0.25f)

#define D2_X0_M ( 0.48f)
#define D2_Y0_M ( 0.10f)

#define D3_X0_M ( 1.80f)
#define D3_Y0_M (-0.73f)

#endif
