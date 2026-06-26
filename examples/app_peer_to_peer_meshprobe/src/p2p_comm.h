#ifndef P2P_COMM_H
#define P2P_COMM_H

#include <stdbool.h>
#include <stdint.h>

#include "p2p_packets.h"

void p2pCommInit(uint8_t my_radio_id);

void p2pCommSendBeacon(const msg_beacon_t* m);
void p2pCommSendClaim(const msg_claim_t* m);
void p2pCommSendDone(const msg_done_t* m);
void p2pCommSendBidVec(const msg_bidvec_t* m);

bool p2pCommPollEvent(app_rx_event_t* out_evt);
void p2pCommTick(void);

void p2pCommSetLocalPos(float x_m, float y_m);
bool p2pCommGetPeerPos(uint8_t peer_radio_id, float* x_m, float* y_m, uint32_t* age_ms);
bool p2pCommGetPeerMetric(uint8_t peer_radio_id, uint16_t* total_dist_cm, uint8_t* done_count,
                          uint8_t* claim_loss_count, uint32_t* age_ms);
bool p2pCommGetPeerStartInfo(uint8_t peer_radio_id, uint8_t* app_state, uint8_t* start_ready,
                             uint32_t* age_ms);

uint32_t p2pCommGetLastRxMs(uint8_t peer_radio_id);
uint32_t p2pCommGetRxCount(void);
uint32_t p2pCommGetDropCount(void);

#endif
