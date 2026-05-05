# app_peer_to_peer_meshprobe 온보딩 가이드

이 예제는 Crazyflie 여러 대가 P2P 무선으로 서로의 상태를 주고받으며, CBBA 기반으로 작업을 분산 할당하고 실제 비행 경로를 따라가며 task 완료를 퍼뜨리는 데모입니다. 단순히 메시지를 보내는 샘플이 아니라, "통신, 상태 동기화, 분산 할당, 비행 제어"가 한 앱 안에서 어떻게 연결되는지를 보여주는 교육용 코드라고 보면 됩니다.

## 목차

1. [이 예제의 목적](#1-이-예제의-목적)
2. [폴더 구조를 읽는 법](#2-폴더-구조를-읽는-법)
3. [전체 실행 흐름](#3-전체-실행-흐름)
4. [상태 기계](#4-상태-기계)
5. [통신 레이어](#5-통신-레이어)
6. [CBBA 로직의 핵심](#6-cbba-로직의-핵심)
7. [스냅샷과 observer](#7-스냅샷과-observer)
8. [실제 비행 제어](#8-실제-비행-제어)
9. [설정값 안내](#9-설정값-안내)
10. [신입이 자주 헷갈리는 지점](#10-신입이-자주-헷갈리는-지점)
11. [코드 읽는 순서 추천](#11-코드-읽는-순서-추천)
12. [실험/디버깅 팁](#12-실험-디버깅-팁)
13. [한 줄 요약](#13-한-줄-요약)

## 1. 이 예제의 목적

핵심 목적은 3대의 드론이 같은 작업 집합을 공유하면서도, 서로 충돌하지 않게 task를 나눠 갖고 수행하는 것입니다. 각 드론은 자기 위치를 기준으로 task에 대한 선호도(bid)를 계산하고, P2P 메시지로 claim/done/snapshot 정보를 교환합니다. 그 결과 전체 팀이 task를 중복 수행하지 않도록 수렴하는지 확인할 수 있습니다.

이 프로젝트를 처음 보는 사람은 아래 4가지만 먼저 기억하면 됩니다.

1. [src/app_main.c](src/app_main.c)는 앱의 진입점이고, State Machine와 비행 제어를 담당합니다.
2. [src/p2p_comm.c](src/p2p_comm.c)는 P2P 수신 콜백, 중복 제거, 이벤트 큐, 송신 래퍼를 담당합니다.
3. [src/cbba_full.c](src/cbba_full.c)는 분산 task 할당 로직과 비행 중 재할당, done 판정을 담당합니다.
4. [src/app_config.h](src/app_config.h)는 실험 파라미터와 좌표, 타이밍, 제한값을 모아둔 설정 파일입니다.

**CBBA 상세 설명**: [CBBA_OVERVIEW.md](CBBA_OVERVIEW.md)를 참고하세요. CBBA의 bid 계산, bundle/path 관리, 메시지 흐름, 수렴 메커니즘을 자세히 다룹니다.

## 2. 폴더 구조를 읽는 법

이 예제 폴더는 대략 다음 역할로 나뉩니다.

- `app_main.c`: 시스템 상태 관리, 비행 세트포인트 생성, P2P 이벤트 소비
- `p2p_comm.c/.h`: P2P 패킷 수신/송신, 최근 수신 시각, 드롭 카운트
- `cbba_full.c/.h`: CBBA 기반 task 할당과 경로 계산
- `cbba_state.c/.h`: 더 단순한 상태/관찰용 구현과 observer용 캐시
- `p2p_packets.h`: 실제 송수신하는 메시지 구조체 정의
- `ids.h`: D1/D2/D3 노드 ID와 이름 변환
- `app_config.h`: 실험에서 자주 바꾸는 상수

빌드 연결은 [Makefile](Makefile)과 [src/Kbuild](src/Kbuild)가 담당합니다. 이 예제는 firmware의 out-of-tree app 형태로 붙기 때문에, 기본적으로 메인 firmware 빌드 시스템을 따라갑니다.

## 3. 전체 실행 흐름

실행 순서는 다음처럼 이해하면 됩니다.

1. 부팅 후 잠시 대기합니다.
2. 자신의 radio low byte를 읽어서 D1/D2/D3 중 누구인지 판별합니다.
3. P2P 통신을 초기화하고 CBBA 상태를 초기화합니다.
4. 다른 두 드론이 충분히 살아 있는지 확인합니다.
5. 세 대가 안정적으로 보이면 takeoff 상태로 진입합니다.
6. RUN 상태에서는 P2P 이벤트를 처리하고, claim/done/snapshot을 송신하면서 task를 수행합니다.
7. 모든 task가 완료되거나 peer를 오래 잃으면 LAND로 전환합니다.
8. 착륙 후 z가 충분히 낮아지면 종료 상태로 들어갑니다.

즉, 이 앱은 "비행 State Machine"와 "분산 task 할당"이 함께 돌고 있습니다. 한쪽만 보면 이해가 안 되고, 두 개를 같이 봐야 합니다.

## 4. State Machine

[src/app_main.c](src/app_main.c) 안에는 아래 상태가 있습니다.

- `ST_IDLE`: 다른 두 드론이 살아 있는지 기다림
- `ST_TAKEOFF`: 고도 상승과 초기 안정화
- `ST_RUN`: task 수행과 P2P 동기화
- `ST_LAND`: 착륙 속도로 하강
- `ST_OFF`: 종료

상태 전이의 포인트는 다음입니다.

- `ST_IDLE -> ST_TAKEOFF`: 3대가 일정 시간 이상 함께 살아 있을 때
- `ST_TAKEOFF -> ST_RUN`: takeoff 시간이 지난 후
- `ST_RUN -> ST_LAND`: peer를 일정 시간 이상 잃었을 때, 또는 전체 task 완료 후 hold 시간이 지난 때
- `ST_LAND -> ST_OFF`: 현재 z 높이가 충분히 낮을 때

이 구조 덕분에 메시지나 할당 로직이 잠깐 흔들려도 비행 자체는 안전하게 마무리됩니다.

## 5. 통신 레이어

[src/p2p_packets.h](src/p2p_packets.h)에는 이 앱이 쓰는 메시지 타입이 정의되어 있습니다.

- `MSG_BEACON`: 존재 확인용 주기 메시지
- `MSG_CLAIM`: 특정 task에 대한 bid 주장
- `MSG_DONE`: 특정 task 완료 알림
- `MSG_SNAPSHOT_FR`: 상태 스냅샷을 조각 단위로 전송

[src/p2p_comm.c](src/p2p_comm.c)에서 중요한 점은 다음입니다.

- 수신 콜백에서 메시지의 `type`, `src`, `seq`를 보고 중복을 제거합니다.
- 최근 수신 시각을 peer별로 저장해 alive 판단에 씁니다.
- 실제 앱 로직으로는 바로 넣지 않고, `app_rx_event_t` 큐에 넣은 뒤 메인 루프가 꺼내 처리합니다.
- 송신은 `p2pCommSendBeacon`, `p2pCommSendClaim`, `p2pCommSendDone`, `p2pCommSendSnapshotFrag`로 감싼 얇은 래퍼입니다.

이 구조의 장점은 P2P 콜백과 앱 상태 갱신을 분리할 수 있다는 점입니다. 수신 ISR/콜백에서 무거운 작업을 하지 않고, main loop에서 순차적으로 처리합니다.

## 6. CBBA 로직의 핵심

이 예제의 본체는 [src/cbba_full.c](src/cbba_full.c)입니다. 여기서는 각 드론이 task에 대한 bid를 계산하고, 가장 적합한 task를 자기 bundle/path에 넣는 방식으로 분산 할당을 수행합니다.

### 6-1. bid 계산

각 task는 좌표를 가지고 있고, 드론은 자기 현재 위치에서 그 task까지의 거리 변화량으로 bid를 계산합니다. 거리가 더 유리할수록 bid가 높아집니다. 그래서 가까운 드론이 자연스럽게 그 task를 맡게 됩니다.

### 6-2. bundle과 path

- `bundle`: 내가 맡을 후보 task 목록
- `path`: 실제 수행 순서
- `exec_task`: 지금 실행 중인 task

bundle은 "내가 이 task들을 할 수 있다"는 집합이고, path는 "실제로 어떤 순서로 갈 것인가"입니다. CBBA는 단순히 가장 비싼 task 하나만 고르는 것이 아니라, 경로 삽입 비용까지 고려해 순서를 잡습니다.

### 6-3. 충돌 해결

claim 메시지를 받으면 task별 winner, bid, version을 비교합니다. 새 claim이 더 최신이거나 더 좋은 bid면 갱신합니다. done 메시지를 받으면 그 task를 완료로 고정하고 bundle/path에서 제거합니다.

### 6-4. local step과 재계산

`Cbba_LocalStep`는 현재 bundle과 winner가 어긋나면 suffix를 잘라내고, 다시 넣을 수 있는 task를 bundle에 채웁니다. 이 부분이 분산 알고리즘의 "수렴"을 담당합니다.

### 6-5. 실제 완료 판정

`Cbba_MarkReachedDone`는 드론이 task 위치 근처에 충분히 오래 머물면 그 task를 done으로 바꿉니다. 즉, 위치 도달만으로 끝내지 않고 dwell time까지 넣어 오탐을 줄입니다.

## 7. 스냅샷과 observer

이 예제는 단순 claim/done만 보내는 것이 아니라, 상태를 잘게 쪼갠 snapshot fragment도 보냅니다. 이유는 전체 team의 수렴 상태를 로그로 관찰하기 쉽도록 하기 위해서입니다.

[src/app_main.c](src/app_main.c)에서는 snapshot fragment를 받으면 `updatePeerCache`로 peer1/peer2 캐시에 넣습니다. 그 뒤 `Cbba_GetObserverMetrics`와 `Cbba_DebugPrintTables`를 통해 다음을 확인합니다.

- 각 드론의 winner table이 얼마나 일치하는지
- 아직 contested task가 남아 있는지
- 각 드론의 실행 task가 일관적인지
- 전체 done count가 runtime task 수에 도달했는지

즉, 이 코드는 "작업 수행"과 "알고리즘 수렴 관찰"을 분리해 둔 예제입니다.

## 8. 실제 비행 제어

[src/app_main.c](src/app_main.c)에서 드론의 실제 움직임을 만듭니다.

- TAKEOFF 동안은 목표 고도를 `TAKEOFF_Z_M`로 둡니다.
- RUN 동안은 현재 실행 task의 목표 좌표로 이동합니다.
- 목표에 가까우면 position hold, 멀면 velocity chase를 사용합니다.
- 속도는 `XY_VEL_MAX`로 제한합니다.
- LAND 동안은 아래 방향 velocity를 줍니다.

이 방식은 초보자가 이해하기 쉬운 편입니다. 즉, 복잡한 경로 추종기보다 task 좌표를 향해 단순 제어를 먼저 구현한 구조입니다.

## 9. 설정값을 먼저 봐야 하는 이유

[src/app_config.h](src/app_config.h)는 이 앱의 실험 성격을 가장 잘 보여줍니다.

중요한 값은 다음입니다.

- `NODE_ID_D1/D2/D3`: 드론 식별자
- `TASK_COUNT_RUNTIME`: 실제 runtime task 수
- `BUNDLE_LIMIT`: 한 번에 잡는 task 수
- `BEACON_TX_HZ`: beacon 송신 주기
- `SNAPSHOT_TX_PERIOD_MS`: snapshot 송신 주기
- `DONE_DWELL_MS`: 완료 판정에 필요한 체류 시간
- `START_HOLD_MS`: 세 대가 안정적으로 모여 있어야 takeoff하는 시간
- `TAKEOFF_Z_M`: 목표 고도
- `TASK0_X_M`~`TASK6_Y_M`: task 좌표

새로 들어온 사람이 가장 먼저 바꿔볼 만한 것은 task 좌표와 타이밍입니다. 다만 통신 타이밍과 완료 판정 조건은 서로 얽혀 있으므로, 한 번에 너무 많이 바꾸면 수렴이 흔들릴 수 있습니다.

## 10. 신입이 자주 헷갈리는 지점

### 10-1. 왜 radio low byte를 노드 ID로 쓰나

이 예제는 Crazyflie의 radio address 하위 1바이트를 드론 식별자로 해석합니다. 그래서 각각의 기체가 서로 다른 ID를 가져야 하며, 같은 채널과 서로 다른 ID 조합이 필요합니다.

### 10-2. 왜 USB 연결을 권장하나

README에서도 설명하듯이, P2P는 무선에서 비동기적으로 동작합니다. PC와 radio link를 같이 쓰면 실험이 불안정해질 수 있어서, 이 예제는 USB 연결을 권장합니다.

### 10-3. 왜 state와 CBBA가 분리되어 있나

비행 안전과 분산 할당은 실패 양상이 다릅니다. 그래서 app_main은 "언제 뜨고 내릴지"를 담당하고, cbba_full은 "무슨 task를 할지"를 담당합니다. 이 분리가 유지보수에서 중요합니다.

## 11. 코드 읽는 순서 추천

신입 온보딩 기준으로는 아래 순서가 가장 빠릅니다.

1. [README.md](README.md)로 예제 목적을 먼저 파악한다.
2. [src/app_config.h](src/app_config.h)로 숫자와 좌표를 본다.
3. [src/p2p_packets.h](src/p2p_packets.h)로 메시지 종류를 익힌다.
4. [src/p2p_comm.c](src/p2p_comm.c)로 수신 큐가 어떻게 돌아가는지 본다.
5. [src/cbba_full.c](src/cbba_full.c)로 task 할당과 완료 처리를 본다.
6. [src/app_main.c](src/app_main.c)로 전체 State Machine와 비행 제어를 연결한다.

## 12. 실험/디버깅 팁

- 세 대 모두 같은 채널이어야 합니다.
- 세 대 모두 서로 다른 ID여야 합니다.
- 시작 위치가 너무 멀면 beacon은 보여도 peer alive 판정이 잘 안 맞을 수 있습니다.
- 로그에서 `OBS_ONE`, `OBS_DETAIL`, `GUIDE`, `DONE_LOCAL`을 보면 알고리즘 수렴과 실제 이동을 같이 추적할 수 있습니다.
- `p2pCommGetDropCount()`가 올라가면 RX 큐가 부족하거나 처리 주기가 늦다는 뜻일 수 있습니다.

## 13. 한 줄 요약

이 예제는 "Crazyflie 여러 대가 P2P로 상태를 공유하면서 CBBA로 task를 나누고, 비행 State Machine 안에서 그 task를 실제로 수행하는 구조"를 보여주는 온보딩용 샘플입니다. 처음에는 통신만 보지 말고, `app_main -> p2p_comm -> cbba_full -> app_main`으로 이어지는 흐름을 따라가면 이해가 빨라집니다.
