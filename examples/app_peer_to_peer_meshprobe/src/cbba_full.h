#ifndef CBBA_FULL_H
#define CBBA_FULL_H

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"
#include "p2p_packets.h"

typedef struct
{
  float x_m;
  float y_m;
} CbbaVec2;

typedef struct
{
  bool active;
  CbbaVec2 pos;
} CbbaTask;

typedef struct
{
  uint8_t agent_id;
  uint8_t task_count;
  uint8_t bundle_limit;

  CbbaVec2 self_pos;
  CbbaTask tasks[TASK_MAX];

  uint8_t done[TASK_MAX];
  uint8_t winner[TASK_MAX];
  int16_t bid_q[TASK_MAX];
  uint8_t ver[TASK_MAX];

  uint8_t bundle[TASK_MAX];
  uint8_t path[TASK_MAX];
  uint8_t bundle_len;
  uint8_t path_len;

  uint8_t exec_task;

  uint8_t tx_seq;
  uint8_t claim_rr;
  uint8_t snap_frag_rr;

  uint32_t last_claim_tx_ms;
  uint32_t last_snapshot_tx_ms;
  uint32_t last_done_tx_ms;
  uint32_t done_enter_ms[TASK_MAX];

  uint32_t local_fp;
  uint8_t contested_count;
  uint8_t done_count;

  uint32_t replan_hold_until_ms;
  uint32_t mission_done_since_ms;
} CbbaState;

typedef struct
{
  uint8_t valid;
  uint8_t src_id;
  uint8_t frag_count;
  uint8_t got_mask;
  uint8_t task_count_total;
  uint8_t exec_task;

  uint8_t done[TASK_MAX];
  uint8_t winner[TASK_MAX];
  int16_t bid_q[TASK_MAX];
  uint8_t ver[TASK_MAX];
} PeerSnapshotCache;

typedef struct
{
  uint8_t all_known;
  uint8_t equal_winner_tasks;
  uint8_t contested_tasks;
  uint8_t winner_conv;
  uint8_t fp_conv;
  uint32_t fp_shadow[AGENT_COUNT];
  uint8_t exec_shadow[AGENT_COUNT];
} CbbaObserverMetrics;

void Cbba_Init(CbbaState *s, uint8_t agent_id, CbbaVec2 start_pos);
void Cbba_SetPose(CbbaState *s, CbbaVec2 pos);

void Cbba_HandleClaim(CbbaState *s, const msg_claim_t *m);
void Cbba_HandleDone(CbbaState *s, const msg_done_t *m);
void Cbba_HandleSnapshotFrag(CbbaState *s, const msg_snapshot_frag_t *m);

void Cbba_LocalStep(CbbaState *s, uint32_t now_ms);
void Cbba_MarkReachedDone(CbbaState *s, uint32_t now_ms);

bool Cbba_MakeClaimMsg(CbbaState *s, uint32_t now_ms, msg_claim_t *out);
bool Cbba_MakeDoneMsg(CbbaState *s, uint32_t now_ms, msg_done_t *out);
bool Cbba_MakeSnapshotFragMsg(CbbaState *s, uint32_t now_ms, msg_snapshot_frag_t *out);

void Cbba_InitPeerCache(PeerSnapshotCache *c);
void Cbba_UpdatePeerCacheFromFrag(PeerSnapshotCache *c, const msg_snapshot_frag_t *m);

void Cbba_GetObserverMetrics(const CbbaState *self,
                             const PeerSnapshotCache *peer1,
                             const PeerSnapshotCache *peer2,
                             CbbaObserverMetrics *out);

uint8_t Cbba_GetGlobalDoneCount(const CbbaState *self,
                                const PeerSnapshotCache *peer1,
                                const PeerSnapshotCache *peer2);

void Cbba_DebugPrintTables(const char *tag,
                           const CbbaState *self,
                           const PeerSnapshotCache *peer1,
                           const PeerSnapshotCache *peer2);

#endif
