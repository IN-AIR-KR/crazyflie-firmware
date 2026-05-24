#ifndef P2P_PACKETS_H
#define P2P_PACKETS_H

#include <stdint.h>

#include "app_config.h"

#define MSG_BEACON 1u
#define MSG_SNAPSHOT_FR 2u

/*
 * Important compatibility rule for the existing visualizer:
 * The legacy packet prefix is preserved:
 *   type, src_id, tx_id, seq, ttl, hop, t_ms, x/y/z, tx_x/tx_y
 * Packet-level forwarding is NOT used in this version, but ttl/hop remain in
 * the wire format so the Python/HTML visualizer can still parse p.hop and
 * position offsets correctly.  For all locally generated packets:
 *   ttl = 1, hop = 0, src_id = tx_id.
 */
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

  /* Appended after the legacy visualizer fields. */
  uint8_t app_state;
  uint8_t cbba_ready;
  uint8_t cbba_started;
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

  uint8_t frag_idx;
  uint8_t frag_count;
  uint8_t task_start_idx;
  uint8_t task_count_total;
  uint8_t task_count_in_frag;
  uint8_t exec_task;

  /* Choi-Brunet-How CBBA timestamp vector s_i.
   * stamp[0], stamp[1], stamp[2] correspond to D1, D2, D3.
   * This is elapsed CBBA mission time after the all-ready barrier, not boot time.
   */
  uint16_t stamp[AGENT_COUNT];

  /* Execution-aware extension.
   * done_mask_global carries the full task-completion vector on every CBBA
   * snapshot fragment.  This avoids waiting for a specific task fragment before
   * completed tasks are removed from re-auction candidates.
   * done_mask_local is retained as a per-fragment mirror for debug/backward
   * compatibility, but the protocol uses done_mask_global as the authoritative
   * DONE state.
   */
  uint16_t done_mask_global;

  /* Compact DONE provenance: 2 bits per task.
   * 00=no validated owner, 01=D1, 10=D2, 11=D3.
   * A receiver accepts a DONE bit only when the corresponding owner field is
   * a valid agent ID.  This keeps DONE inside the CBBA packet while avoiding
   * the large done_owner[TASK_MAX] array that overloaded the radio path.
   */
  uint32_t done_owner_bits;

  uint8_t done_mask_local;
  uint8_t winner[SNAP_FRAG_TASKS];
  int16_t bid_q[SNAP_FRAG_TASKS];

  /* Debug-only local task version. It is not used by CBBA Table-I logic. */
  uint8_t ver[SNAP_FRAG_TASKS];
} msg_snapshot_frag_t;

typedef struct {
  uint8_t type;
  union {
    msg_snapshot_frag_t snapf;
  } u;
} app_rx_event_t;

#endif
