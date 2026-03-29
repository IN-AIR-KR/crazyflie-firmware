#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "radiolink.h"
#include "configblock.h"

// 위치 데이터 타입 및 스테빌라이저 API 헤더
#include "stabilizer_types.h"
#include "stabilizer.h" 

#define DEBUG_MODULE "P2P_DIST"
#include "debug.h"

// 패킷 구조: [ID(1)][X(4)][Y(4)][Z(4)] = 13 bytes
#define P2P_PAYLOAD_SIZE 13

/**
 * 수신 콜백 함수: 패킷 수신 시 거리 계산 및 출력
 */
void p2pcallbackHandler(P2PPacket *p)
{
  uint8_t other_id = p->data[0];
  float rxX, rxY, rxZ;

  // 1. 상대방 좌표 추출 (바이트 -> float)
  memcpy(&rxX, &p->data[1], 4);
  memcpy(&rxY, &p->data[5], 4);
  memcpy(&rxZ, &p->data[9], 4);

  // 2. 내 현재 위치 가져오기
  state_t my_state;
  stabilizerGetState(&my_state); 

  float myX = my_state.position.x;
  float myY = my_state.position.y;
  float myZ = my_state.position.z;

  // 3. 유클리드 거리 계산: $d = \sqrt{(x_1-x_2)^2 + (y_1-y_2)^2 + (z_1-z_2)^2}$
  float dx = myX - rxX;
  float dy = myY - rxY;
  float dz = myZ - rxZ;
  float distance = sqrtf(dx*dx + dy*dy + dz*dz);

  // 4. 시각화된 콘솔 출력
  DEBUG_PRINT("============================================================\n");
  DEBUG_PRINT("📡 [From CF %d] RSSI: -%d dBm\n", other_id, p->rssi);
  DEBUG_PRINT("   📍 SELF : (%6.2f, %6.2f, %6.2f)\n", (double)myX, (double)myY, (double)myZ);
  DEBUG_PRINT("   📍 PEER : (%6.2f, %6.2f, %6.2f)\n", (double)rxX, (double)rxY + 0.3, (double)rxZ);
  DEBUG_PRINT("   📏 DIST :  >>> %.2f m <<<\n", (double)distance);

  if (distance < 0.5f && distance > 0.01f) {
    DEBUG_PRINT("   ⚠️  WARNING: COLLISION HAZARD! ⚠️\n");
  }
}

/**
 * 앱 메인 루프
 */
void appMain()
{
  DEBUG_PRINT("P2P Distance Monitoring Started!\n");

  static P2PPacket p_packet;
  p_packet.port = 0x00;

  // 1. 내 드론의 고유 ID 설정 (라디오 주소 끝자리 활용)
  uint64_t address = configblockGetRadioAddress();
  uint8_t my_id = (uint8_t)(address & 0x00000000ff);
  p_packet.data[0] = my_id;

  // 2. 콜백 함수 등록
  p2pRegisterCB(p2pcallbackHandler);

  state_t current_state;

  while(1) {
    vTaskDelay(M2T(200)); // 5Hz 주기로 방송 (0.2초마다)

    // 3. 현재 위치 정보 획득
    stabilizerGetState(&current_state);

    float px = current_state.position.x;
    float py = current_state.position.y;
    float pz = current_state.position.z;

    // 4. 패킷 구성
    memcpy(&p_packet.data[1], &px, 4);
    memcpy(&p_packet.data[5], &py, 4);
    memcpy(&p_packet.data[9], &pz, 4);
    p_packet.size = P2P_PAYLOAD_SIZE; 
    
    // 5. 브로드캐스트 전송
    radiolinkSendP2PPacketBroadcast(&p_packet);
  }
}
