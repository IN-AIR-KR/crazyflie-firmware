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
  int16_t value_q;
  CbbaVec2 pos;
  CbbaVec2 exit_pos;
} CbbaTask;

typedef struct {
  uint8_t agent_id;
  uint8_t task_count;
  uint8_t bundle_limit;

  CbbaVec2 self_pos;
  CbbaTask tasks[TASK_MAX];

  uint8_t done[TASK_MAX];
  uint16_t done_mask;
  uint8_t winner[TASK_MAX];
  int16_t bid_q[TASK_MAX];
  uint8_t ver[TASK_MAX];

  uint8_t bundle[TASK_MAX];
  uint8_t path[TASK_MAX];
  uint8_t bundle_len;
  uint8_t path_len;

  uint8_t exec_task;

  uint8_t tx_seq;
  uint8_t bidvec_seq;
  uint8_t claim_rr;
  uint8_t done_rr;

  uint32_t last_claim_tx_ms;
  uint32_t last_bidvec_tx_ms;
  uint32_t last_bid_refresh_ms;
  uint32_t last_done_tx_ms;
  uint32_t replan_hold_until_ms;
  uint32_t done_enter_ms[TASK_MAX];
  uint32_t winner_rx_ms[TASK_MAX];
  uint8_t claim_burst_left;
  uint8_t lane_entered[TASK_MAX];

  uint32_t local_fp;
  uint8_t contested_count;
  uint8_t done_count;
  uint8_t local_done_count;

  uint32_t mission_done_since_ms;

  int16_t peer_bid_q[AGENT_COUNT][TASK_MAX];
  uint8_t peer_exec_task[AGENT_COUNT];
  uint16_t peer_done_mask[AGENT_COUNT];
  uint32_t peer_bid_rx_ms[AGENT_COUNT];
  uint8_t peer_bid_valid[AGENT_COUNT];

  /* Runtime metrics for P2P vs Mesh comparison. */
  uint16_t claim_rx_count;
  uint16_t claim_loss_count;
  uint16_t late_release_count;
  uint16_t bidvec_rx_count;
  uint16_t auction_switch_count;
} CbbaState;

typedef struct {
  uint8_t valid;
  uint8_t src_id;
  uint8_t task_count_total;
  uint8_t exec_task;

  uint8_t done[TASK_MAX];
  uint16_t done_mask;
  uint8_t winner[TASK_MAX];
  int16_t bid_q[TASK_MAX];
  uint8_t ver[TASK_MAX];
} PeerMissionCache;

void Cbba_Init(CbbaState* s, uint8_t agent_id, CbbaVec2 start_pos);
void Cbba_SetPose(CbbaState* s, CbbaVec2 pos);

void Cbba_HandleClaim(CbbaState* s, const msg_claim_t* m, uint32_t now_ms);
void Cbba_HandleDone(CbbaState* s, const msg_done_t* m, uint32_t now_ms);
void Cbba_HandleBidVec(CbbaState* s, const msg_bidvec_t* m, uint32_t now_ms);

void Cbba_LocalStep(CbbaState* s, uint32_t now_ms);
void Cbba_MarkReachedDone(CbbaState* s, uint32_t now_ms);
bool Cbba_GetExecTarget(const CbbaState* s, CbbaVec2* out_target, uint8_t* out_phase);

bool Cbba_MakeClaimMsg(CbbaState* s, uint32_t now_ms, msg_claim_t* out);
bool Cbba_MakeDoneMsg(CbbaState* s, uint32_t now_ms, msg_done_t* out);
bool Cbba_MakeBidVecMsg(CbbaState* s, uint32_t now_ms, msg_bidvec_t* out);

void Cbba_InitPeerCache(PeerMissionCache* c);
void Cbba_UpdatePeerCacheFromClaim(PeerMissionCache* c, const msg_claim_t* m);
void Cbba_UpdatePeerCacheFromDone(PeerMissionCache* c, const msg_done_t* m);
void Cbba_UpdatePeerCacheFromBidVec(PeerMissionCache* c, const msg_bidvec_t* m);

void Cbba_AbsorbGlobalDoneMask(CbbaState* self, const PeerMissionCache* peer1,
                               const PeerMissionCache* peer2, uint32_t now_ms);

uint16_t Cbba_GetGlobalDoneMask(const CbbaState* self, const PeerMissionCache* peer1,
                                const PeerMissionCache* peer2);
uint8_t Cbba_GetGlobalDoneCount(const CbbaState* self, const PeerMissionCache* peer1,
                                const PeerMissionCache* peer2);

#endif
