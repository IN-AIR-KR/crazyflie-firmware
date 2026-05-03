/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * peer_to_peer.c - App layer application of simple demonstration peer to peer
 *  communication. Two crazyflies need this program in order to send and
 * receive.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app.h"
#include "configblock.h"
#include "p2p_packets.h"
#include "radiolink.h"
#include "task.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"

void p2pcallbackHandler(P2PPacket* p) {
  if (p->size < sizeof(msg_beacon_t)) {
    return;
  }

  msg_beacon_t beacon;
  memcpy(&beacon, p->data, sizeof(beacon));
  if (beacon.type != MSG_BEACON) {
    return;
  }

  DEBUG_PRINT(
      "[RX RSSI:-%u dBm] beacon src:%u tx:%u seq:%u ttl:%u hop:%u t_ms:%u\n",
      p->rssi, beacon.src_id, beacon.tx_id, beacon.seq, beacon.ttl, beacon.hop,
      beacon.t_ms);
}

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  static P2PPacket p_reply;
  static msg_beacon_t tx_beacon;

  p_reply.port = 0x00;
  p_reply.size = sizeof(msg_beacon_t);

  uint64_t address = configblockGetRadioAddress();
  uint8_t my_id = (uint8_t)((address) & 0x00000000ff);

  tx_beacon.type = MSG_BEACON;
  tx_beacon.src_id = my_id;
  tx_beacon.tx_id = my_id;
  tx_beacon.seq = 0u;
  tx_beacon.ttl = 1u;
  tx_beacon.hop = 0u;
  tx_beacon.t_ms = 0u;

  p2pRegisterCB(p2pcallbackHandler);

  while (1) {
    tx_beacon.tx_id = my_id;
    tx_beacon.seq++;
    tx_beacon.ttl = 1u;
    tx_beacon.hop = 0u;
    tx_beacon.t_ms = (uint16_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    memcpy(p_reply.data, &tx_beacon, sizeof(tx_beacon));
    radiolinkSendP2PPacketBroadcast(&p_reply);
    DEBUG_PRINT("[TX] beacon src:%u tx:%u seq:%u ttl:%u hop:%u t_ms:%u\n",
                tx_beacon.src_id, tx_beacon.tx_id, tx_beacon.seq, tx_beacon.ttl,
                tx_beacon.hop, tx_beacon.t_ms);

    vTaskDelay(M2T(500));
  }
}
