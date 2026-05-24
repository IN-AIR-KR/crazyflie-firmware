#ifndef CBBA_FULL_H
#define CBBA_FULL_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "p2p_packets.h"

typedef struct {
  float x_m;
  float y_m;
} CbbaVec2;

typedef struct {
  bool active;
  CbbaVec2 pos;
} CbbaTask;

typedef struct {
  uint8_t agent_id;
  uint8_t task_count;
  uint8_t bundle_limit;

  CbbaVec2 self_pos;
  CbbaTask tasks[TASK_MAX];

  uint8_t done[TASK_MAX];
  uint16_t done_mask;

  /* USE_DONE_PROTOCOL=1 only.  Compact owner/provenance for DONE bits.
   * 2 bits per task: 0=none, 1=D1, 2=D2, 3=D3.
   */
  uint32_t done_owner_bits;

  /* Local execution-only progress. Used by USE_DONE_PROTOCOL=0 so that
   * each drone can fly through its frozen CBBA path without creating a
   * global completion consensus state.
   */
  uint8_t local_exec_done[TASK_MAX];
  uint16_t local_exec_done_mask;
  uint8_t local_exec_done_count;

  uint8_t winner[TASK_MAX];
  int16_t bid_q[TASK_MAX];

  /* Debug-only local task version. Not used in Table-I conflict resolution. */
  uint8_t ver[TASK_MAX];

  uint8_t bundle[TASK_MAX];
  uint8_t path[TASK_MAX];
  uint8_t bundle_len;
  uint8_t path_len;

  uint8_t exec_task;

  uint8_t tx_seq;
  uint8_t snap_frag_rr;

  uint32_t last_snapshot_tx_ms;
  uint8_t force_snapshot_tx;

  /* USE_DONE_PROTOCOL=1: DONE reliability layer.
   * pending_done_mask contains DONE bits that this node will keep
   * retransmitting in CBBA SNAPSHOT_FR until every non-lost agent has echoed
   * the same DONE bit back. peer_done_echo[i] is the latest ORed DONE mask
   * known to be echoed by agent i.
   */
  uint16_t pending_done_mask;
  uint16_t peer_done_echo[AGENT_COUNT];

  /* USE_DONE_PROTOCOL=1: newly selected claims during execution are not
   * chased until the surviving peers have echoed the same winner.  This is
   * not a fixed-time delay; it is a CBBA y/z echo gate for new residual-task
   * assignments after DONE/failure updates.
   */
  uint16_t pending_claim_mask;

  uint32_t done_enter_ms[TASK_MAX];

  uint32_t local_fp;
  uint8_t contested_count;
  uint8_t done_count;

  uint32_t replan_hold_until_ms;
  uint32_t mission_done_since_ms;

  /* USE_DONE_PROTOCOL=0 freezes the paper-CBBA allocation after the
   * pre-execution settling phase. The frozen path is then executed locally.
   */
  uint8_t allocation_frozen;
  uint32_t allocation_freeze_ms;

  /* USE_DONE_PROTOCOL=1 only: failed agents whose unfinished tasks have
   * already been released for re-auction.
   */
  uint8_t lost_agent_mask;

  /* Choi-Brunet-How CBBA timestamp vector s_i.
   * stamp[0], stamp[1], stamp[2] are D1/D2/D3 information times.
   * They are elapsed CBBA mission timestamps after the all-ready barrier,
   * not boot-time values.
   */
  uint16_t stamp[AGENT_COUNT];
  uint8_t cbba_started;
  uint32_t cbba_epoch_ms;
} CbbaState;

typedef struct {
  uint8_t valid;
  uint8_t src_id;
  uint8_t frag_count;
  uint8_t got_mask;
  uint8_t task_count_total;
  uint8_t exec_task;

  uint8_t done[TASK_MAX];
  uint16_t done_mask;

  /* USE_DONE_PROTOCOL=1 only.  Compact owner/provenance for DONE bits.
   * 2 bits per task: 0=none, 1=D1, 2=D2, 3=D3.
   */
  uint32_t done_owner_bits;

  uint8_t winner[TASK_MAX];
  int16_t bid_q[TASK_MAX];
  uint8_t ver[TASK_MAX];
  uint16_t stamp[AGENT_COUNT];
} PeerSnapshotCache;

typedef struct {
  uint8_t all_known;
  uint8_t equal_winner_tasks;
  uint8_t contested_tasks;
  uint8_t winner_conv;
  uint8_t fp_conv;
  uint32_t fp_shadow[AGENT_COUNT];
  uint8_t exec_shadow[AGENT_COUNT];
} CbbaObserverMetrics;

void Cbba_Init(CbbaState* s, uint8_t agent_id, CbbaVec2 start_pos);
void Cbba_StartMission(CbbaState* s, uint32_t now_ms);
void Cbba_SetPose(CbbaState* s, CbbaVec2 pos);
void Cbba_FreezeAllocation(CbbaState* s, uint32_t now_ms);
uint8_t Cbba_IsAllocationFrozen(const CbbaState* s);
uint8_t Cbba_IsLocalExecutionFinished(const CbbaState* s);
uint8_t Cbba_GetLocalExecutionDoneCount(const CbbaState* s);
uint8_t Cbba_HasPendingDone(const CbbaState* s);
uint8_t Cbba_CanMoveToExec(const CbbaState* self,
                           const PeerSnapshotCache* peer1,
                           const PeerSnapshotCache* peer2);
uint8_t Cbba_DeclareAgentLost(CbbaState* s, uint8_t lost_agent,
                              uint32_t now_ms);

void Cbba_HandleSnapshotFrag(CbbaState* s, const msg_snapshot_frag_t* m,
                             uint32_t now_ms);

void Cbba_LocalStep(CbbaState* s, uint32_t now_ms);
void Cbba_MarkReachedDone(CbbaState* s, uint32_t now_ms);

bool Cbba_MakeSnapshotFragMsg(CbbaState* s, uint32_t now_ms,
                              msg_snapshot_frag_t* out);

void Cbba_InitPeerCache(PeerSnapshotCache* c);
void Cbba_UpdatePeerCacheFromFrag(PeerSnapshotCache* c,
                                  const msg_snapshot_frag_t* m);

void Cbba_GetObserverMetrics(const CbbaState* self,
                             const PeerSnapshotCache* peer1,
                             const PeerSnapshotCache* peer2,
                             CbbaObserverMetrics* out);

uint16_t Cbba_GetGlobalDoneMask(const CbbaState* self,
                                const PeerSnapshotCache* peer1,
                                const PeerSnapshotCache* peer2);
uint8_t Cbba_GetGlobalDoneCount(const CbbaState* self,
                                const PeerSnapshotCache* peer1,
                                const PeerSnapshotCache* peer2);

void Cbba_DebugPrintTables(const char* tag, const CbbaState* self,
                           const PeerSnapshotCache* peer1,
                           const PeerSnapshotCache* peer2);

#endif
