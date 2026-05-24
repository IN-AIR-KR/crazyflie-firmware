#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define NODE_ID_D1 0xE6u
#define NODE_ID_D2 0xE7u
#define NODE_ID_D3 0xE8u

#define AGENT_COUNT 3u

/* ============================================================
 * Communication mode
 * ------------------------------------------------------------
 * This version intentionally uses P2P-only neighbor communication.
 * There is no packet forwarding layer.
 *
 * CBBA graph-wide information propagation occurs only through repeated
 * neighbor-to-neighbor SNAPSHOT exchange:
 *   D1 snapshot -> D2 merges y/z/s -> D2 next snapshot -> D3 merges y/z/s
 *
 * BEACON  : alive / synchronization / visualization
 * SNAPSHOT: CBBA y_i, z_i, s_i, done table fragment
 * ============================================================ */
#define USE_RANGE_LIMIT 0u
#define USE_BEACON_RANGE_LIMIT 0u
#define USE_CBBA_RANGE_LIMIT 0u

#define BEACON_RADIUS_M 4.50f  /* ignored when USE_BEACON_RANGE_LIMIT=0 */
#define CBBA_COMM_RADIUS_M 1.05f

/* ============================================================
 * Connectivity-Constrained CBBA switch
 * ------------------------------------------------------------
 * 0u: baseline distance-only task feasibility.
 * 1u: connectivity-preserving frontier CBBA.
 *
 * This is an allocation feasibility constraint, not packet forwarding.
 * The only model parameter is the communication-link radius used
 * for predicted graph connectivity checks.
 * ============================================================ */
#define USE_CONNECTIVITY_CONSTRAINT 0u
#define CONNECTIVITY_RADIUS_M 1.05f

/* ============================================================
 * DONE protocol experiment switch
 * ------------------------------------------------------------
 * 0u: comparison baseline. Paper-style CBBA allocation is performed
 *     during the hover/settling phase, then the assigned path is frozen.
 *     Each drone advances through only its own local path. No global DONE
 *     bits are transmitted, no remote DONE is accepted, and no failed-winner
 *     recovery is executed. This keeps the CBBA allocation logic intact while
 *     omitting the execution-completion protocol that is absent from the
 *     original CBBA paper.
 *
 * 1u: proposed execution-aware mode. DONE bits are transmitted in
 *     SNAPSHOT_FR, completed tasks are removed globally, and a failed agent's
 *     unfinished tasks are reset for re-auction.
 * ============================================================ */
#define USE_DONE_PROTOCOL 1u
#define LOST_AGENT_RELEASE_MS PEER_LOSS_STREAK_MS

/* mission switch */
#define MISSION_AUTO_START 1u
// 0 = P2P communication only, no takeoff
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
#define PEER_LOSS_STREAK_MS 1000u
#define START_HOLD_MS 3000u
#define TAKEOFF_MS 1800u
#define TAKEOFF_Z_M 0.60f
#define LAND_VZ_MPS (-0.18f)

/* communication / queue */
#define SEEN_N 192u
#define RX_QUEUE_N 64u

/* CBBA communication periods
 * SNAPSHOT is the only CBBA state packet in this version.
 * One full 9-task table cycle takes SNAPSHOT_TX_PERIOD_MS * 3.
 */
#define SNAPSHOT_TX_PERIOD_MS 250u
#define SUMMARY_LOG_MS 1000u
#define STATE_LOG_MS 1000u

/* task table/path/bundle diagnostic log */
#define TASK_TABLE_DEBUG_ENABLE 1u
#define TASK_TABLE_DEBUG_PERIOD_MS 2000u

/* done */
#define DONE_RADIUS_M 0.28f
#define DONE_DWELL_MS 250u

/* near-goal position hold */
#define GOAL_HOLD_RADIUS_M 0.25f

/* no immediate task chase after takeoff */
#define POST_TAKEOFF_HOLD_MS 3800u

/* Start the CBBA all-ready handshake while the drones are still hovering.
 * This gives the y/z/s consensus several snapshot cycles before XY motion.
 */
#define CBBA_READY_AFTER_RUN_MS 500u
#define CBBA_ASSIGN_SETTLE_MS 2500u

/* Paper-CBBA start barrier.
 * Each drone sets cbba_ready=1 after the post-takeoff hover hold.
 * CBBA bundle construction and timestamp epoch start only after all ready
 * flags are observed continuously for this duration.
 */
#define CBBA_READY_HOLD_MS 1000u

/* Paper-CBBA timestamp unit for s_i.
 * The timestamp is elapsed mission time after Cbba_StartMission(),
 * not the Crazyflie boot time.
 */
#define CBBA_STAMP_UNIT_MS 100u

/* Paper-compatible time-discounted reward scoring.
 * Score for task j in a path uses c_bar * exp(-alpha * arrival_time).
 * The marginal score is S(path with j inserted) - S(path).
 */
#define CBBA_SCORING_SPEED_MPS 0.20f
#define CBBA_DISCOUNT_ALPHA_PER_SEC 0.035f
#define CBBA_TASK_REWARD_Q 10000.0f

/* XY velocity control */
#define XY_KP 0.8f
#define XY_VEL_MAX 0.19f

/* continuous visit */
#define REPLAN_HOLD_MS 0u

/* Execution-aware reassignment settling.
 * These holds do not change task geometry or scoring. They only block XY
 * chasing briefly after a DONE update or a failed-agent release so that
 * y/z/s snapshots can converge before another drone starts moving to the
 * same newly freed task.
 */
#define POST_DONE_ASSIGN_SETTLE_MS 500u
#define RECOVERY_ASSIGN_SETTLE_MS 0u

/* final DONE spreading hold */
#define MISSION_DONE_HOLD_MS 5000u

/* ============================================================
 * Urban information-collection task layout
 * Coordinate convention:
 *   +x : forward
 *   +y : left
 *
 * Baseline expected motion when USE_CONNECTIVITY_CONSTRAINT=0:
 *   D1 -> left upper urban tasks
 *   D2 -> nearby lower task first
 *   D3 -> right upper urban tasks
 *
 * Proposed expected motion when USE_CONNECTIVITY_CONSTRAINT=1:
 *   Frontier 1: D1->T1, D2->T0, D3->T2
 *   Frontier 2: D1->T6, D2->T8, D3->T7
 *   Frontier 3: D1->T3, D2->T5, D3->T4
 *   All 9 tasks are still visited; connectivity only changes order.
 * ============================================================ */
#define D1_X0_M (-0.75f)
#define D1_Y0_M ( 0.00f)

#define D2_X0_M ( 0.00f)
#define D2_Y0_M ( 0.00f)

#define D3_X0_M ( 0.75f)
#define D3_Y0_M ( 0.00f)

/* ============================================================
 * Wide asymmetric task layout, scaled by 0.8 from previous layout
 * ------------------------------------------------------------
 * Original scale: x,y roughly within [-1.75, 1.75]
 * New scale    : x,y roughly within [-1.40, 1.40]
 * ============================================================ */

/* Task 0 */
#define TASK0_X_M ( 1.24f)
#define TASK0_Y_M ( 0.68f)

/* Task 1 */
#define TASK1_X_M (-1.16f)
#define TASK1_Y_M ( 1.04f)

/* Task 2 */
#define TASK2_X_M ( 0.28f)
#define TASK2_Y_M ( 1.36f)

/* Task 3 */
#define TASK3_X_M (-1.36f)
#define TASK3_Y_M (-0.16f)

/* Task 4 */
#define TASK4_X_M ( 1.00f)
#define TASK4_Y_M (-1.16f)

/* Task 5 */
#define TASK5_X_M (-0.20f)
#define TASK5_Y_M (-1.40f)

/* Task 6 */
#define TASK6_X_M ( 1.40f)
#define TASK6_Y_M ( 0.04f)

/* Task 7 */
#define TASK7_X_M (-1.44f)
#define TASK7_Y_M ( 0.38f)

/* Task 8 */
#define TASK8_X_M ( 1.36f)
#define TASK8_Y_M (-0.48f)

#endif
