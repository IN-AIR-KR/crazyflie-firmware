#ifndef P2P_COMM_H
#define P2P_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "p2p_packets.h"

void p2pCommInit(uint8_t my_radio_id);

void p2pCommSendBeacon(const msg_beacon_t *m);
void p2pCommSendClaim(const msg_claim_t *m);
void p2pCommSendDone(const msg_done_t *m);
void p2pCommSendSnapshotFrag(const msg_snapshot_frag_t *m);

bool p2pCommPollEvent(app_rx_event_t *out_evt);

uint32_t p2pCommGetLastRxMs(uint8_t peer_radio_id);
uint32_t p2pCommGetRxCount(void);
uint32_t p2pCommGetDropCount(void);

#endif
