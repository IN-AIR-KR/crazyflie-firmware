#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define NODE_ID_D1 0xE6u
#define NODE_ID_D2 0xE7u
#define NODE_ID_D3 0xE8u

#define AGENT_COUNT 3u

/* CBBA 통신 거리 제한 (beacon 패킷은 항상 거리 제한 없음) */
#define USE_CBBA_RANGE_LIMIT 0u
#define CBBA_COMM_RADIUS_M 2.0f

/* mission switch */
#define MISSION_AUTO_START 0u
// 0 = mesh 통신만 테스트(이륙 안 함)
// 1 = 전원 연결 후 자동 이륙+미션

/* CBBA 크기 */
#define TASK_MAX 8u
#define TASK_COUNT_RUNTIME 7u
#define BUNDLE_LIMIT 3u

/* 비행 / 상태 */
#define LOOP_HZ 50u
#define BEACON_TX_HZ 20u
#define CBBA_TX_PERIOD_MS 250u
#define PEER_TO_MS 5000u
#define PEER_LOSS_STREAK_MS 3000u
#define START_HOLD_MS 3000u
#define TAKEOFF_MS 1200u
#define TAKEOFF_Z_M 0.60f
#define LAND_VZ_MPS (-0.18f)

/* 통신 / 큐 */
#define SEEN_N 128u
#define RX_QUEUE_N 12u

/* 진단 로그 주기 */
#define SUMMARY_LOG_MS 1000u
#define STATE_LOG_MS 1000u
#define TASK_TABLE_DEBUG_ENABLE 1u
#define TASK_TABLE_DEBUG_PERIOD_MS 2000u

/* done 판정 완화 */
#define DONE_RADIUS_M 0.18f
#define DONE_DWELL_MS 250u

/* 목표 근처에서 velocity chase 대신 position hold */
#define GOAL_HOLD_RADIUS_M 0.25f

/* TAKEOFF 후 바로 task chase 금지 */
#define POST_TAKEOFF_HOLD_MS 1800u

/* XY 속도 제한 */
#define XY_KP 0.8f
#define XY_VEL_MAX 0.18f

/* 연속 방문 목적 */
#define REPLAN_HOLD_MS 0u

/* 마지막 완료 상태를 모두에게 퍼뜨릴 시간 */
#define MISSION_DONE_HOLD_MS 1500u

#endif
