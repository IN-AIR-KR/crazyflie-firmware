#ifndef CBBA_STATE_H
#define CBBA_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "mesh_packets.h"

typedef struct
{
  uint8_t id;
  uint8_t my_idx;

  bool done[APP_NUM_TASKS];
  uint8_t owner[APP_NUM_TASKS];
  int16_t owner_bid[APP_NUM_TASKS];

  uint8_t bundle[APP_BUNDLE_SIZE];
  uint8_t bundle_len;
  uint8_t exec_task;
  uint8_t claim_rr;

  uint8_t evt_id;
  uint8_t tx_seq;

  uint32_t last_claim_tx_ms;
  uint32_t last_snapshot_tx_ms;
  uint32_t exec_start_ms;
  uint32_t last_summary_ms;

  uint32_t local_table_fp;
  uint32_t apply_count;
  uint32_t stale_count;

  /* observer shadow */
  bool seen_snapshot[APP_NUM_DRONES];
  uint32_t fp_shadow[APP_NUM_DRONES];
  uint8_t packed_winners_shadow[APP_NUM_DRONES][3];
  uint8_t done_bits_shadow[APP_NUM_DRONES][2];
  uint8_t bundle_len_shadow[APP_NUM_DRONES];
  uint8_t bundle_shadow[APP_NUM_DRONES][APP_BUNDLE_SIZE];
  uint8_t exec_task_shadow[APP_NUM_DRONES];
  uint8_t last_evt_shadow[APP_NUM_DRONES];

  bool first_seen_evt_valid[APP_NUM_DRONES][EVT_SLOTS];
  uint32_t first_seen_evt_ms[APP_NUM_DRONES][EVT_SLOTS];
} cbba_state_t;

void cbbaInit(cbba_state_t *s, uint8_t myId);
void cbbaStepLocal(cbba_state_t *s, uint32_t nowMs);

void cbbaOnClaim(cbba_state_t *s, const msg_claim_t *m);
void cbbaOnDone(cbba_state_t *s, const msg_done_t *m);
void cbbaOnSnapshot(cbba_state_t *s, const msg_snapshot_t *m, uint32_t nowMs);

bool cbbaMakeClaim(cbba_state_t *s, msg_claim_t *outMsg);
bool cbbaMakeSnapshot(cbba_state_t *s, msg_snapshot_t *outMsg);
bool cbbaMakeDone(cbba_state_t *s, msg_done_t *outMsg);

void cbbaPrintObserverSummary(cbba_state_t *s, uint32_t nowMs);

#endif
