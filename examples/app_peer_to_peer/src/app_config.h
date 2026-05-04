#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define NODE_ID_D1 0xE6u
#define NODE_ID_D2 0xE7u
#define NODE_ID_D3 0xE8u

#define AGENT_COUNT               3u

/* direct-only */
#define USE_MESH                  0u
#define USE_RANGE_LIMIT           0u
#define COMM_RADIUS_M             1.00f
#define TTL_MAX                   1u

/* CBBA 크기 */
#define TASK_MAX                  8u
#define TASK_COUNT_RUNTIME        7u
#define BUNDLE_LIMIT              3u

/* snapshot fragment */
#define SNAP_FRAG_TASKS           3u
#define SNAP_FRAG_COUNT           ((TASK_COUNT_RUNTIME + SNAP_FRAG_TASKS - 1u) / SNAP_FRAG_TASKS)

/* 비행 / 상태 */
#define LOOP_HZ                   50u
#define BEACON_TX_HZ              20u
#define PEER_TO_MS                5000u
#define PEER_LOSS_STREAK_MS       3000u
#define START_HOLD_MS             3000u
#define TAKEOFF_MS                1200u
#define TAKEOFF_Z_M               0.60f
#define LAND_VZ_MPS              (-0.18f)

/* 통신 / 큐 */
#define SEEN_N                    128u
#define RX_QUEUE_N                24u

/* CBBA 통신 주기 */
#define SNAPSHOT_TX_PERIOD_MS     250u
#define CLAIM_TX_PERIOD_MS        150u
#define DONE_REPEAT_PERIOD_MS     150u
#define SUMMARY_LOG_MS            1000u
#define STATE_LOG_MS              1000u

/* task table/path/bundle 진단 로그: 조건 없이 항상 출력 */
#define TASK_TABLE_DEBUG_ENABLE   1u
#define TASK_TABLE_DEBUG_PERIOD_MS 2000u

/* done 판정 완화 */
#define DONE_RADIUS_M             0.18f
#define DONE_DWELL_MS             250u

/* 목표 근처에서는 velocity chase 대신 position hold */
#define GOAL_HOLD_RADIUS_M        0.25f

/* TAKEOFF 후 바로 task chase 금지 */
#define POST_TAKEOFF_HOLD_MS      1800u

/* XY 속도 제한 접근 */
#define XY_KP                     0.8f
#define XY_VEL_MAX                0.18f

/* 연속 방문 목적 */
#define REPLAN_HOLD_MS            0u

/* 마지막 완료 상태를 모두에게 퍼뜨릴 시간 */
#define MISSION_DONE_HOLD_MS      1500u

/* ===== 7개 task : 중심(0,0), x/y 약 ±1.2m 이내 ===== */
#define TASK0_X_M  -0.90f
#define TASK0_Y_M  -0.85f

#define TASK1_X_M  -0.30f
#define TASK1_Y_M   1.05f

#define TASK2_X_M   0.95f
#define TASK2_Y_M   0.80f

#define TASK3_X_M   1.10f
#define TASK3_Y_M  -0.35f

#define TASK4_X_M  -1.15f
#define TASK4_Y_M   0.25f

#define TASK5_X_M   0.20f
#define TASK5_Y_M  -1.10f

#define TASK6_X_M   0.70f
#define TASK6_Y_M   0.05f

/* ===== 실제 시작 배치 ===== */
#define D1_X0_M    -0.35f
#define D1_Y0_M    -0.20f

#define D2_X0_M     0.00f
#define D2_Y0_M     0.20f

#define D3_X0_M     0.35f
#define D3_Y0_M    -0.20f

#endif
