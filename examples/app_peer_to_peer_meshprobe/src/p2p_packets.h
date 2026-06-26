#ifndef P2P_PACKETS_H
#define P2P_PACKETS_H

#include <stdint.h>

#include "app_config.h"

#define MSG_BEACON 1u
#define MSG_CLAIM 2u
#define MSG_DONE 3u
#define MSG_BIDVEC 5u

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t src_id;
  uint8_t tx_id;
  uint8_t seq;
  uint8_t ttl;
  uint8_t hop;
  uint16_t t_ms;
  int16_t x_cm;
  int16_t y_cm;
  int16_t z_cm;
  int16_t tx_x_cm;
  int16_t tx_y_cm;
  uint16_t total_dist_cm;
  uint8_t done_count;
  uint8_t claim_loss_count;
  uint8_t app_state;
  uint8_t start_ready;
} msg_beacon_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t src_id;
  uint8_t tx_id;
  uint8_t seq;
  uint8_t ttl;
  uint8_t hop;

  int16_t tx_x_cm;
  int16_t tx_y_cm;

  uint8_t task_id;
  int16_t bid_q;
  uint8_t ver;
  uint8_t path_idx;
  uint16_t heard_mask;
  uint16_t done_mask;
} msg_claim_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t src_id;
  uint8_t tx_id;
  uint8_t seq;
  uint8_t ttl;
  uint8_t hop;

  int16_t tx_x_cm;
  int16_t tx_y_cm;

  uint8_t task_id;
  uint8_t ver;
  uint16_t done_mask;
} msg_done_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t src_id;
  uint8_t tx_id;
  uint8_t seq;
  uint8_t ttl;
  uint8_t hop;

  int16_t tx_x_cm;
  int16_t tx_y_cm;

  uint16_t done_mask;
  uint8_t task_count;
  uint8_t exec_task;
  int16_t bid_q[TASK_MAX];
} msg_bidvec_t;

typedef struct {
  uint8_t type;
  union {
    msg_claim_t claim;
    msg_done_t done;
    msg_bidvec_t bidv;
  } u;
} app_rx_event_t;

#endif
