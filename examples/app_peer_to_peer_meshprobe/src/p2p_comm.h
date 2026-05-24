#ifndef P2P_COMM_H
#define P2P_COMM_H

#include <stdbool.h>
#include <stdint.h>

#include "p2p_packets.h"

void p2pCommInit(uint8_t my_radio_id);

void p2pCommSendBeacon(const msg_beacon_t* m);
void p2pCommSendCbbaState(const msg_cbba_state_t* m);

bool p2pCommPollCbbaState(msg_cbba_state_t* out);

void p2pCommSetLocalPos(float x_m, float y_m);
uint32_t p2pCommGetLastRxMs(uint8_t peer_radio_id);
uint32_t p2pCommGetRxCount(void);

#endif
