#!/usr/bin/env python3
"""
Crazyflie P2P 메시 비콘 데이터 수신 & 파싱 예제

동작 방식:
  1. Crazyflie(그라운드 스테이션)가 다른 Crazyflie의 P2P 브로드캐스트를 수신
  2. app_main.c의 p2pcallbackHandler가 CRTP 포트 0x09로 PC에 포워딩
  3. 이 스크립트가 CRTP 포트 0x09 콜백으로 수신 & 파싱

필요 조건:
  - Crazyflie 1대: app_peer_to_peer_mesh 펌웨어 설치 후 PC에 연결 (그라운드 역할)
  - Crazyflie 1대 이상: 공중에서 P2P 비콘 브로드캐스트 (송신 역할)

msg_beacon_t 구조체 (8 bytes):
  uint8_t  type   (1 byte)
  uint8_t  src_id (1 byte)
  uint8_t  tx_id  (1 byte)
  uint8_t  seq    (1 byte)
  uint8_t  ttl    (1 byte)
  uint8_t  hop    (1 byte)
  uint16_t t_ms   (2 bytes, little-endian)

설치:
    pip install cflib

실행:
    python3 beacon_receiver.py [radio://0/80/2M]
"""

import sys
import time
import struct
from datetime import datetime
import cflib.crtp
from cflib.crazyflie import Crazyflie


# 메시지 타입 정의
MSG_BEACON = 1
MSG_TYPES = {
    1: "BEACON",
}


class BeaconParser:
    """msg_beacon_t 구조체 파싱 클래스"""

    # 구조체 포맷: B=uint8, H=uint16 (little-endian)
    # msg_beacon_t: type(B) src_id(B) tx_id(B) seq(B) ttl(B) hop(B) t_ms(H) = 8 bytes
    STRUCT_FORMAT = '<BBBBBBH'  # <는 little-endian (6개 B + 1개 H)
    STRUCT_SIZE = 8  # bytes

    @staticmethod
    def parse(data):
        """바이트 데이터를 msg_beacon_t 구조체로 파싱

        Args:
            data: 바이트 배열

        Returns:
            dict: 파싱된 구조체 필드
        """
        if len(data) < BeaconParser.STRUCT_SIZE:
            return None

        try:
            # 구조체 언팩
            values = struct.unpack(BeaconParser.STRUCT_FORMAT, data[:BeaconParser.STRUCT_SIZE])

            # 필드 매핑
            beacon = {
                'type': values[0],
                'src_id': values[1],
                'tx_id': values[2],
                'seq': values[3],
                'ttl': values[4],
                'hop': values[5],
                't_ms': values[6],
            }

            return beacon
        except struct.error as e:
            print(f"파싱 오류: {e}")
            return None

    @staticmethod
    def get_type_name(type_id):
        """타입 ID를 문자열로 변환"""
        return MSG_TYPES.get(type_id, f"UNKNOWN({type_id})")


class BeaconReceiver:
    def __init__(self, uri='radio://0/80/2M'):
        self.uri = uri
        self.cf = Crazyflie(rw_cache='./cache')

        # 연결 콜백
        self.cf.connected.add_callback(self._on_connected)
        self.cf.disconnected.add_callback(self._on_disconnected)
        self.cf.connection_failed.add_callback(self._on_connection_failed)

        # 패킷 수신 콜백: 펌웨어가 CRTP_PORT_P2P_PROXY(0x09)로 포워딩한 P2P 데이터
        self.cf.incoming.add_port_callback(0x09, self._on_packet_received)

        self.packet_count = 0
        self.beacon_count = 0

    def _on_packet_received(self, packet):
        """라디오 패킷 수신 콜백"""
        self.packet_count += 1

        # packet.data는 바이트 배열
        if not packet.data:
            return

        # 구조체 파싱
        beacon = BeaconParser.parse(packet.data)
        if not beacon:
            return

        self.beacon_count += 1

        # 타입 확인
        if beacon['type'] != MSG_BEACON:
            print(f"✗ 알 수 없는 타입: {beacon['type']}")
            return

        # 데이터 출력
        self._print_beacon(beacon, packet.data)

    def _print_beacon(self, beacon, data):
        """비콘 데이터를 포맷해서 출력"""
        timestamp = time.strftime('%H:%M:%S')

        # 기본 정보
        print(f"\n{'='*80}")
        print(f"[{timestamp}] 비콘 수신 #{self.beacon_count}")
        print(f"{'='*80}")

        # 라디오 정보
        print(f"\n📡 라디오 정보:")
        print(f"   Port: 0x00 (P2P 메시지)")
        print(f"   Data Size: {len(data)} bytes")

        # 비콘 필드
        print(f"\n📦 비콘 구조체:")
        print(f"   ┌─ 타입 (type)")
        print(f"   │  ├─ 값: {beacon['type']}")
        print(f"   │  └─ 의미: {BeaconParser.get_type_name(beacon['type'])}")

        print(f"   ├─ 송신자 ID (src_id)")
        print(f"   │  ├─ 값: {beacon['src_id']}")
        print(f"   │  └─ 16진수: 0x{beacon['src_id']:02x}")

        print(f"   ├─ 트랜스미터 ID (tx_id)")
        print(f"   │  ├─ 값: {beacon['tx_id']}")
        print(f"   │  └─ 16진수: 0x{beacon['tx_id']:02x}")

        print(f"   ├─ 시퀀스 번호 (seq)")
        print(f"   │  └─ 값: {beacon['seq']}")

        print(f"   ├─ TTL (Time To Live)")
        print(f"   │  └─ 값: {beacon['ttl']} 홉")

        print(f"   ├─ 홉 카운트 (hop)")
        print(f"   │  └─ 값: {beacon['hop']}")

        print(f"   └─ 타임스탬프 (t_ms)")
        print(f"      ├─ 값: {beacon['t_ms']} ms")
        print(f"      └─ 시간: {beacon['t_ms']/1000:.3f} 초")

        # 16진수 덤프
        print(f"\n💾 16진수 덤프:")
        hex_str = ' '.join(f'{b:02x}' for b in data[:BeaconParser.STRUCT_SIZE])
        print(f"   {hex_str}")

        # 해석
        print(f"\n🔍 해석:")
        print(f"   송신자 {beacon['src_id']:02x}가 보낸 패킷입니다.")
        print(f"   시퀀스: {beacon['seq']}, TTL: {beacon['ttl']}, 홉: {beacon['hop']}")

    def _on_connected(self, uri):
        """연결 성공"""
        print(f"\n✓ 라디오 연결 성공!")
        print(f"  URI: {uri}")
        print("=" * 80)
        print("비콘 수신 중 (P2P Mesh 메시지)...")
        print("Ctrl+C를 눌러 종료합니다.\n")

    def _on_disconnected(self, uri):
        """연결 해제"""
        print(f"\n✗ 라디오 연결 해제")

    def _on_connection_failed(self, uri, msg):
        """연결 실패"""
        print(f"\n✗ 라디오 연결 실패: {msg}")

    def connect(self):
        """라디오 연결"""
        print(f"라디오 연결 중... URI: {self.uri}")
        self.cf.open_link(self.uri)

    def disconnect(self):
        """라디오 연결 해제"""
        self.cf.close_link()

    def is_connected(self):
        """연결 상태"""
        return self.cf.is_connected()


def main():
    """메인 함수"""
    if len(sys.argv) > 1:
        uri = sys.argv[1]
    else:
        uri = 'radio://0/80/2M'

    print("=" * 80)
    print("Crazyflie P2P 메시 비콘 수신 & 파싱")
    print("=" * 80)
    print()
    print("📝 이 예제는:")
    print("   - app_peer_to_peer_mesh 펌웨어의 비콘 메시지를 수신합니다")
    print("   - msg_beacon_t 구조체를 파싱하고 분석합니다")
    print("   - 각 필드의 의미를 설명합니다")
    print()
    print("✅ 사전 준비:")
    print("   1. Crazyflie에 app_peer_to_peer_mesh 펌웨어 설치")
    print("   2. 라디오 동글 USB 연결")
    print("   3. Crazyflie 전원 ON")
    print()
    print(f"📡 URI: {uri}\n")

    cflib.crtp.init_drivers()

    receiver = BeaconReceiver(uri)

    try:
        receiver.connect()

        # 연결 대기
        timeout = 5
        start_time = time.time()
        while not receiver.is_connected() and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        if not receiver.is_connected():
            print(f"✗ 오류: {timeout}초 이내에 연결되지 않았습니다.")
            print("\n도움말:")
            print("  1. 라디오 동글이 USB에 연결되어 있는지 확인")
            print("  2. Crazyflie가 켜져 있고 라디오 범위 내에 있는지 확인")
            print("  3. app_peer_to_peer_mesh 펌웨어가 설치되어 있는지 확인")
            return 1

        # 계속 수신
        while receiver.is_connected():
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n" + "=" * 80)
        print("수신 중지")
        print("=" * 80)
    finally:
        receiver.disconnect()
        print("\n✓ 라디오 연결 해제")
        print(f"총 수신: {receiver.packet_count}개 패킷, {receiver.beacon_count}개 비콘")

    return 0


if __name__ == '__main__':
    sys.exit(main())
