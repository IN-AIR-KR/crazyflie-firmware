#ifndef P2P_PACKETS_H
#define P2P_PACKETS_H

#include <stdint.h>
#include "app_config.h"

#define MSG_BEACON       1u
#define MSG_CLAIM        2u
#define MSG_DONE         3u
#define MSG_SNAPSHOT_FR  4u

typedef struct __attribute__((packed))
{
  uint8_t type;
  uint8_t src_id;
  uint8_t tx_id;
  uint8_t seq;
  uint8_t ttl;
  uint8_t hop;
  uint16_t t_ms;
} msg_beacon_t;

typedef struct __attribute__((packed))
{
  uint8_t type;
  uint8_t src_id;
  uint8_t tx_id;
  uint8_t seq;
  uint8_t ttl;
  uint8_t hop;

  uint8_t task_id;
  int16_t bid_q;
  uint8_t ver;
  uint8_t path_idx;
} msg_claim_t;

typedef struct __attribute__((packed))
{
  uint8_t type;
  uint8_t src_id;
  uint8_t tx_id;
  uint8_t seq;
  uint8_t ttl;
  uint8_t hop;

  uint8_t task_id;
  uint8_t ver;
} msg_done_t;

typedef struct __attribute__((packed))
{
  uint8_t type;
  uint8_t src_id;
  uint8_t tx_id;
  uint8_t seq;
  uint8_t ttl;
  uint8_t hop;

  uint8_t frag_idx;
  uint8_t frag_count;
  uint8_t task_start_idx;
  uint8_t task_count_total;
  uint8_t task_count_in_frag;
  uint8_t exec_task;

  uint8_t done_mask_local;                 /* fragment 내 로컬 bitmask */
  uint8_t winner[SNAP_FRAG_TASKS];
  int16_t bid_q[SNAP_FRAG_TASKS];
  uint8_t ver[SNAP_FRAG_TASKS];
} msg_snapshot_frag_t;

typedef struct
{
  uint8_t type;
  union
  {
    msg_claim_t claim;
    msg_done_t done;
    msg_snapshot_frag_t snapf;
  } u;
} app_rx_event_t;

#endif
