# CBAA Formation Flight with APF Collision Avoidance

Crazyflie 3대가 **CBAA**(Consensus-Based Auction Algorithm)로 목표 대형을
자율 배정받고, **APF**(Artificial Potential Field)로 충돌을 회피하며 비행하는 데모.

---

## 목차

1. [시나리오 개요](#1-시나리오-개요)
2. [알고리즘: CBAA](#2-알고리즘-cbaa)
3. [알고리즘: APF](#3-알고리즘-apf)
4. [시스템 아키텍처](#4-시스템-아키텍처)
5. [패킷 구조](#5-패킷-구조)
6. [상태 머신](#6-상태-머신)
7. [CBAA 수렴 흐름 예시](#7-cbaa-수렴-흐름-예시)
8. [주요 Crazyflie API 정리](#8-주요-crazyflie-api-정리)
9. [로그 변수 (cfclient 모니터링)](#9-로그-변수-cfclient-모니터링)
10. [빌드 및 플래싱](#10-빌드-및-플래싱)
11. [파라미터 튜닝 가이드](#11-파라미터-튜닝-가이드)
12. [알려진 제한사항](#12-알려진-제한사항)

---

## 1. 시나리오 개요

```
       y
       ^
  1.0  |        [Task 1]
       |       (0.5, 1.0)
  0.5  |  [Task 0]     [Task 2]
       |  (0.0, 0.5)  (1.0, 0.5)
  0.0  |---[CF1]---[CF2]---[CF3]---> x
           (0.0)  (0.5)  (1.0)

  시작: 직선 대형 (y=0)
  목표: 삼각형 대형 (z=0.5m 고도)
```

| 단계          | 내용                                                       |
| ------------- | ---------------------------------------------------------- |
| **이륙**      | 3대 동시 이륙 → 0.5m 호버링                                |
| **CBAA 합의** | P2P 브로드캐스트로 각자 목표 위치 경매·합의 (비행 중 계속) |
| **APF 비행**  | 배정 목표로 이동. 충돌 위험 시 척력으로 경로 보정          |
| **수렴 판정** | 배정 5회 안정 + 목적지 15cm 이내 도착 → 대형 형성 완료 |
| **착륙**      | 3초 호버링 후 착륙                                         |

**예상 최적 배정:**

```
CF1 (0.0,0.0) → Task 0 (0.0,0.5)  : 가장 가까운 거리 0.5m
CF2 (0.5,0.0) → Task 1 (0.5,1.0)  : 경쟁 후 배정
CF3 (1.0,0.0) → Task 2 (1.0,0.5)  : 가장 가까운 거리 0.5m
```

---

## 2. 알고리즘: CBAA

> 참고: Choi et al., _"Consensus-Based Decentralized Auctions for Robust Task Allocation,"_
> IEEE Trans. Robotics, Vol.25, No.4, 2009.

CBAA는 **경매(Auction)** 와 **합의(Consensus)** 두 단계를 반복해 분산 배정을 달성한다.
중앙 서버 없이 이웃 에이전트끼리 브로드캐스트만으로 수렴한다.

### 2.1 상태 변수 (에이전트 i 기준)

| 변수          | 타입  | 설명                                  |
| ------------- | ----- | ------------------------------------- |
| `x_assign[j]` | uint8 | 배정 벡터. `1` = 내가 task j 담당     |
| `y_bid[j]`    | float | task j의 현재 낙찰 최고가 (단조 증가) |
| `z_winner[j]` | uint8 | task j의 현재 낙찰자 ID               |
| `c[j]`        | float | task j에 대한 나의 점수 (동적 갱신)   |

### 2.2 점수 함수

```
c_ij = 1 / (1 + dist_xy(현재위치_i, task_j))

  - 비음수 조건(논문 c_ij ≥ 0) 만족
  - 가까울수록 → 1.0 (최고)
  - 멀수록    → 0.0 (하한)
  - 분모 최소 1.0 → ÷0 없음
  - 매 반복 현재 위치 기반 재계산 → 비행 중 실시간 갱신
```

### 2.3 Phase 1: 경매 (Auction)

```
미배정(Σ x_assign = 0) 인 경우에만 실행:

  유효 태스크 집합: H = { j | c[j] > y_bid[j] }
                          (내 점수가 현재 낙찰가보다 높은 태스크)

  낙찰 태스크: J* = argmax_{j ∈ H} c[j]

  갱신:
    x_assign[J*] = 1
    y_bid[J*]    = c[J*]
    z_winner[J*] = my_id
```

### 2.4 Phase 2: 합의 (Consensus)

이웃 k의 패킷 `(y_k, z_k)` 수신 시:

```
for each task j:
    if y_k[j] > y_bid[j]:          # 이웃이 더 높은 입찰가 보유
        y_bid[j]    = y_k[j]
        z_winner[j] = z_k[j]
        if z_winner[j] ≠ my_id:    # 내가 낙찰자가 아니면
            x_assign[j] = 0        # 배정 취소 → 다음 경매에서 재입찰
```

### 2.5 수렴 조건

```
완전 연결 3노드 네트워크의 직경 D=1.
이론적 수렴: D+1 = 2회 반복.

구현 판정 (두 조건 동시 충족):
  1. CBAA 안정: assigned_task가 N_STABLE_REQUIRED=5 회 연속 불변
  2. 도착 확인: dist(현재위치, TASK_POS[assigned_task]) < ARRIVAL_THRESHOLD(0.15m)

  → 루프가 계속 GoTo를 재발행하면서 비행하다가,
     목적지에 실제로 도착하고 배정도 안정되면 탈출.
  → 타임아웃(MAX_CBAA_ITER=100) 시에는 루프 탈출 후 최종 GoTo(5s)로 보정.

┌──────────┐     ┌──────────┐     ┌──────────┐
│  CF1     │────>│  CF2     │────>│  CF3     │
│(브로드캐스트)│<────│(브로드캐스트)│<────│(브로드캐스트)│
└──────────┘     └──────────┘     └──────────┘
    완전 연결 (모든 쌍이 직접 통신)
```

---

## 3. 알고리즘: APF

APF(Artificial Potential Field)는 **인력**과 **척력**을 합산해 충돌을 회피한다.

### 3.1 힘 구성

```
F_total = F_attractive + F_repulsive

F_attractive: GoTo 명령이 담당 (목표 방향으로 이동)

F_repulsive (이웃 k에 의한 척력):

  dist_k = dist_xy(현재위치, 이웃_k_위치)

  if dist_k < ρ₀:  (영향 반경 내)
      factor = K_rep * (1/dist_k - 1/ρ₀) / dist_k²
      F_rep += factor * (현재위치 - 이웃위치)   # 이웃 반대 방향

  클램프: |F_rep| ≤ APF_MAX_REP
```

### 3.2 APF 보정 목표

```
APF 보정은 xy 평면만 적용 (z는 HL commander가 task_z로 제어):

  apf_target_x = task_x + rep_x
  apf_target_y = task_y + rep_y
  apf_target_z = task_z         (APF 영향 없음)

→ GoTo(apf_target, duration=1.0s) 를 200ms마다 재발행
   (Receding Horizon 방식: 짧은 궤적을 계속 갱신)
```

### 3.3 APF 파라미터

| 파라미터              | 값     | 설명                  |
| --------------------- | ------ | --------------------- |
| `APF_K_REP`           | 0.05   | 척력 게인             |
| `APF_RHO_0`           | 0.6m   | 척력 영향 반경        |
| `APF_MAX_REP`         | 0.3m   | 최대 척력 오프셋      |
| `APF_MIN_DIST`        | 0.15m  | 분모 0 방지 최소 거리 |
| `NEIGHBOR_TIMEOUT_MS` | 1000ms | 위치 유효 시간        |

---

## 4. 시스템 아키텍처

### 4.1 전체 흐름

```
┌─────────────────────────────────────────────────────────────┐
│                      appMain() - FreeRTOS Task               │
│                                                              │
│  ┌──────────┐   ┌──────────┐   ┌────────────────────────┐  │
│  │  TAKEOFF │──>│HOVER_WAIT│──>│       CBAA_FLY          │  │
│  └──────────┘   └──────────┘   │                        │  │
│                                │  매 200ms 반복:          │  │
│                                │  1. 현재 위치 읽기        │  │
│                                │  2. 점수 재계산           │  │
│                                │  3. Phase1: 경매          │  │
│                                │  4. P2P 브로드캐스트      │  │
│                                │  5. 수신 패킷 처리         │  │
│                                │     (Phase2: 합의)        │  │
│                                │  6. APF 목표 계산         │  │
│                                │  7. GoTo(APF목표, 1.0s)  │  │
│                                │  8. 수렴 판정             │  │
│                                │  stable≥5+dist<0.15m    │  │
│                                └──────────┬─────────────┘  │
│                                           |                 │
│  ┌──────────┐   ┌──────────┐             | 수렴            │
│  │  DONE    │<──│   LAND   │<────────────┘                 │
│  └──────────┘   └──────────┘                               │
└─────────────────────────────────────────────────────────────┘
       ^
       | P2P 콜백 (syslinkTask)
       | 수신 패킷 → rx_shadow 버퍼 (taskENTER_CRITICAL 보호)
```

### 4.2 스레드 안전 설계

```
syslinkTask (높은 우선순위)          appMain (낮은 우선순위)
──────────────────────────          ──────────────────────
p2pCallback() 호출                  200ms 폴링 루프
    │                                   │
    ├─ 패킷 검증                         ├─ taskENTER_CRITICAL()
    ├─ taskENTER_CRITICAL()             ├─ got = new_data_ready
    ├─ rx_shadow ← 패킷 복사             ├─ if got: local ← rx_shadow
    ├─ new_data_ready = true            ├─          new_data_ready = false
    └─ taskEXIT_CRITICAL()             └─ taskEXIT_CRITICAL()
                                            │
                                            └─ cbaaConsensus(local)
                                               cbaaAuction()

임계구역: 18바이트 memcpy + bool 플래그 (매우 짧음)
단일 슬롯 섀도우 버퍼: 가장 최근 패킷만 보존 (충분, 합의는 누적됨)
```

### 4.3 CBAA 반복 타임라인

```
시간(ms)
  0  ──────> cbaaUpdateScores()    ← 현재 위치로 점수 갱신
  0  ──────> cbaaAuction()         ← 경매 (미배정 시)
  0  ──────> cbaaBroadcast()       ← P2P 전송
  0  ─ 수신 폴링 시작 (10ms 간격)
 10  ──────> cbaaPollRx()          ┐
 20  ──────> cbaaPollRx()          │ 수신 패킷 → Consensus + Auction
 ...                               │ (최대 19번 폴링)
190  ──────> cbaaPollRx()          ┘
200  ──────> apfCompute()          ← APF 목표 계산
200  ──────> GoTo(apf_target, 1s)  ← 경로 명령 (1초 궤적, 200ms마다 갱신)
200  ──────> 다음 반복 시작
```

---

## 5. 패킷 구조

```
P2PPacket.data[] (최대 60바이트, 실제 사용 28바이트)

Byte  Field        Type      설명
────  ──────────── ────────  ──────────────────────────────────
  0   sender_id    uint8     라디오 주소 하위 바이트 (0xE7/08/09)
 1-4  y[0]         float32   Task 0 낙찰 최고가
 5-8  y[1]         float32   Task 1 낙찰 최고가
9-12  y[2]         float32   Task 2 낙찰 최고가
 13   z_win[0]     uint8     Task 0 낙찰자 ID (0=없음)
 14   z_win[1]     uint8     Task 1 낙찰자 ID
 15   z_win[2]     uint8     Task 2 낙찰자 ID
16-19 pos[0]       float32   현재 x 위치 (m) ← APF용
20-23 pos[1]       float32   현재 y 위치 (m)
24-27 pos[2]       float32   현재 z 위치 (m)
────
합계: 28 bytes  (P2P_MAX_DATA_SIZE=60 이하 ✓)
```

**포트 번호:** `0x01`

- Port 0x00: 기존 peer_to_peer 예제
- Port 0x0F (15): DTR 프로토콜
- Port 0x01: 이 앱 (충돌 없음)

---

## 6. 상태 머신

```
INIT ──> WAIT_PEERS ─── 모든 피어 감지 ──> TAKEOFF
           (500ms마다 브로드캐스트,            │
            peer_heard[k] 모두 true 시 탈출)  이륙 완료
                                              │
                                              v
                                         HOVER_WAIT ── 1초 ──> CBAA_FLY
                                                                    │
                                                            ┌───────┴────────┐
                                                            │  매 200ms 반복   │
                                                            │  CBAA + APF     │
                                                            │  + GoTo(1.0s)   │
                                                            └───────┬────────┘
                                                                    │
                                                              stable≥5 탈출
                                                                    │
                                                            최종 GoTo(5s)
                                                            + 도착 대기
                                                                    │
                                                                    v
                                                               LAND_WAIT ── 3초 ──> LAND
                                                                                       │
                                                                                  착륙 완료
                                                                                       │
                                                                                       v
                                                                                     DONE

상태 번호: 0=INIT 1=WAIT_PEERS 2=TAKEOFF 3=HOVER_WAIT 4=CBAA_FLY 5=LAND_WAIT 6=LAND 7=DONE
```

---

## 7. CBAA 수렴 흐름 예시

초기 상태 (모두 미배정, y_bid=0.0):

```
Iter 0:
  CF1 점수: c0=0.667  c1=0.472  c2=0.472
  CF2 점수: c0=0.586  c1=0.500  c2=0.586
  CF3 점수: c0=0.472  c1=0.472  c2=0.667

  경매 결과:
    CF1 → Task 0 (c0=0.667 최고)  y_bid[0]=0.667, z[0]=0x01
    CF2 → Task 0 (c0=0.586 최고)  y_bid[0]=0.586, z[0]=0x02  ← CF1과 충돌
    CF3 → Task 2 (c2=0.667 최고)  y_bid[2]=0.667, z[2]=0x03

  합의 후:
    CF2가 CF1 브로드캐스트 수신: y_bid[0]=0.667 > 0.586
      → Task 0 낙찰자=CF1 (CF2 패배) → CF2 미배정 상태로 재입찰
    CF2 재입찰 → Task 1 (유일하게 남은 태스크)

Iter 1:
  배정 안정화:
    CF1 → Task 0,  CF2 → Task 1,  CF3 → Task 2

Iter 2~5:
  배정 불변 → stable_count 증가

Iter 5 (stable=5):
  CONVERGED ✓ → 루프 탈출 → 최종 GoTo(5s) → IsTrajectoryFinished() 대기
```

**최종 대형 (위에서 본 모습):**

```
       y
  1.0  |      [CF2]
       |
  0.5  | [CF1]     [CF3]
       |
  0.0  |--CF1--CF2--CF3--> x  (이륙 전)
```

---

## 8. 주요 Crazyflie API 정리

### 8.1 High-Level Commander (`crtp_commander_high_level.h`)

HL commander는 내부적으로 다항식 궤적(polynomial trajectory)을 계획해
부드러운 비행을 보장한다. app 레이어에서 가장 쉽게 자율 비행을 구현할 수 있다.

```c
// ── 이륙 ──────────────────────────────────────────────────
int crtpCommanderHighLevelTakeoff(
    const float absoluteHeight_m,  // 목표 고도 (절대값, m)
    const float duration_s         // 이륙 소요 시간 (s)
);
// 예: crtpCommanderHighLevelTakeoff(0.5f, 2.0f);
//     → 0.5m까지 2초에 걸쳐 이륙

// ── 착륙 ──────────────────────────────────────────────────
int crtpCommanderHighLevelLand(
    const float absoluteHeight_m,  // 목표 고도 (보통 0.0)
    const float duration_s
);

// ── 위치 이동 ──────────────────────────────────────────────
int crtpCommanderHighLevelGoTo(
    const float x,          // 목표 x (m)
    const float y,          // 목표 y (m)
    const float z,          // 목표 z (m)
    const float yaw,        // 목표 yaw (rad)
    const float duration_s, // 이동 소요 시간 (s)
    const bool  relative    // true=상대좌표, false=절대좌표
);
// 예: crtpCommanderHighLevelGoTo(0.5f, 1.0f, 0.5f, 0.0f, 3.0f, false);

// ── 궤적 완료 확인 ─────────────────────────────────────────
bool crtpCommanderHighLevelIsTrajectoryFinished(void);
// 반환값: true = 명령한 궤적이 완료됨 (호버링 중 또는 착륙 완료)
// 사용 패턴:
//   crtpCommanderHighLevelTakeoff(0.5f, 2.0f);
//   while (!crtpCommanderHighLevelIsTrajectoryFinished()) {
//       vTaskDelay(M2T(100));
//   }
```

> **주의:** `GoTo`를 짧은 주기(200ms)로 반복 호출하면 HL commander가
> 계속 새 궤적을 계획한다. `duration_s`를 주기보다 길게(예: 1.0s) 설정해
> 부드러운 움직임을 유지한다 (Receding Horizon 방식).

---

### 8.2 P2P Radio (`radiolink.h`)

```c
// ── 패킷 구조체 ────────────────────────────────────────────
#define P2P_MAX_DATA_SIZE 60

typedef struct _P2PPacket {
    uint8_t size;   // 페이로드 크기 (바이트)
    uint8_t rssi;   // 수신 신호 강도 (수신 시에만 유효)
    union {
        struct {
            uint8_t port;                     // 라우팅 포트 (0~15)
            uint8_t data[P2P_MAX_DATA_SIZE];  // 페이로드
        };
        uint8_t raw[P2P_MAX_DATA_SIZE + 1];
    };
} __attribute__((packed)) P2PPacket;

// ── 브로드캐스트 전송 ──────────────────────────────────────
bool radiolinkSendP2PPacketBroadcast(P2PPacket *p2pp);
// 모든 인접 CF에게 패킷 전송 (Best-effort, ACK 없음)
// 사용 패턴:
//   P2PPacket pkt;
//   pkt.port = 0x01;
//   memcpy(pkt.data, &my_struct, sizeof(my_struct));
//   pkt.size = sizeof(my_struct);
//   radiolinkSendP2PPacketBroadcast(&pkt);

// ── 수신 콜백 등록 ─────────────────────────────────────────
typedef void (*P2PCallback)(P2PPacket *);
void p2pRegisterCB(P2PCallback cb);
// 패킷 수신 시 syslinkTask 컨텍스트에서 cb가 호출됨
// ⚠️ 콜백은 빠르게 처리 → 공유 변수는 taskENTER_CRITICAL()로 보호
```

---

### 8.3 Config Block (`configblock.h`)

```c
// ── 라디오 주소 취득 ───────────────────────────────────────
uint64_t configblockGetRadioAddress(void);
// CF 고유 5바이트 주소 반환. 하위 1바이트가 CF ID로 사용됨.
// 예: 주소 = 0xE7E7E7E701 → ID = 0x01

// 사용 패턴:
//   uint64_t addr  = configblockGetRadioAddress();
//   uint8_t  my_id = (uint8_t)(addr & 0xFF);  // → 0x01, 0x02, 0x03
```

---

### 8.4 Log 시스템 (`log.h`)

앱 레이어에서 Kalman 필터 추정 위치를 읽는 방법:

```c
// ── 로그 변수 ID 취득 (초기화 시 한 번만) ─────────────────
logVarId_t logGetVarId(const char *group, const char *name);
// 예:
//   logVarId_t id_x = logGetVarId("stateEstimate", "x");
//   logVarId_t id_y = logGetVarId("stateEstimate", "y");
//   logVarId_t id_z = logGetVarId("stateEstimate", "z");
// ⚠️ ID가 0xFF이면 변수를 찾지 못한 것 (그룹/이름 오타 확인)

// ── 값 읽기 ────────────────────────────────────────────────
float logGetFloat(logVarId_t varid);
// 예:
//   float pos_x = logGetFloat(id_x);

// ── 앱에서 로그 변수 노출 ──────────────────────────────────
static float my_var = 0.0f;

LOG_GROUP_START(my_group)
LOG_ADD(LOG_FLOAT, my_var_name, &my_var)
LOG_GROUP_STOP(my_group)
// cfclient에서 my_group.my_var_name 으로 구독 가능
```

---

### 8.5 FreeRTOS 유틸리티

```c
// ── 태스크 대기 ────────────────────────────────────────────
void vTaskDelay(TickType_t xTicksToDelay);
// M2T(ms): 밀리초를 FreeRTOS tick으로 변환
// 예: vTaskDelay(M2T(200));  → 200ms 대기

// ── 현재 시각 ──────────────────────────────────────────────
TickType_t xTaskGetTickCount(void);
// M2T(ms) tick 단위 반환. 오버플로우에 주의 (uint32_t, ~49일 주기)

// ── 임계구역 ───────────────────────────────────────────────
taskENTER_CRITICAL();   // 인터럽트/다른 태스크 선점 차단 시작
// ... 공유 변수 접근 (최소한으로 유지) ...
taskEXIT_CRITICAL();    // 선점 재개
// ⚠️ Cortex-M4: configMAX_SYSCALL_INTERRUPT_PRIORITY 이하 인터럽트 차단
```

---

## 9. 로그 변수 (cfclient 모니터링)

cfclient의 Log Configuration에서 `cbaa` 그룹을 구독한다.

| 변수          | 타입  | 설명                                  |
| ------------- | ----- | ------------------------------------- |
| `cbaa.state`  | UINT8 | 앱 상태 (4=CBAA_FLY 확인)             |
| `cbaa.task`   | UINT8 | 배정 태스크 (0xFF=미배정, 0/1/2=배정) |
| `cbaa.iter`   | UINT8 | CBAA 전체 반복 번호                   |
| `cbaa.stable` | UINT8 | 연속 안정 반복 수 (5 이상 → 수렴)     |
| `cbaa.posX`   | FLOAT | 현재 x 위치 (m)                       |
| `cbaa.posY`   | FLOAT | 현재 y 위치 (m)                       |
| `cbaa.apfTx`  | FLOAT | APF 보정 후 목표 x                    |
| `cbaa.apfTy`  | FLOAT | APF 보정 후 목표 y                    |
| `cbaa.score0` | FLOAT | Task 0 점수 c[0]                      |
| `cbaa.score1` | FLOAT | Task 1 점수 c[1]                      |
| `cbaa.score2` | FLOAT | Task 2 점수 c[2]                      |

**검증 체크리스트:**

```
□ 이륙 후: cbaa.state == 4 (CBAA_FLY)
□ 몇 회 반복 후: cbaa.task 고정 (3대 모두 다른 값 0/1/2)
□ 비행 중: cbaa.score* 실시간 변화 확인
□ 수렴 후: cbaa.stable >= 5
□ APF 효과: cbaa.apfTx/Ty 가 task 위치와 약간 다를 때 이웃 CF 근처
□ 착륙 완료: cbaa.state == 7 (DONE)
```

---

## 10. 빌드 및 플래싱

### 빌드

```bash
# crazyflie-firmware 루트에서
cd examples/app_cbaa_formation
make

# 또는 루트에서 직접
make OOT_CONFIG=$(pwd)/examples/app_cbaa_formation/app-config
```

### 플래싱 (cfclient)

```
1. CF를 USB로 연결 (또는 Crazyradio로 연결)
2. cfclient → Bootloader → Flash firmware
3. 생성된 .bin/.elf 파일 선택
4. 3대 모두 같은 펌웨어 플래싱
```

### CF 주소 설정

코드 상단의 `AGENT_IDS` 배열에 실제 기체 주소 하위 바이트를 입력:

```c
/* cbaa_formation.c */
static const uint8_t AGENT_IDS[N_AGENTS] = {0xE7, 0x08, 0x09};
//                                           CF1    CF2    CF3
```

순서는 `START_POS` 배열 순서와 동일해야 함:

- `AGENT_IDS[0]` = 0xE7 → 시작 위치 `START_POS[0]` = (0.0, 0.0) 에 놓인 기체
- `AGENT_IDS[1]` = 0x08 → 시작 위치 `START_POS[1]` = (0.5, 0.0) 에 놓인 기체
- `AGENT_IDS[2]` = 0x09 → 시작 위치 `START_POS[2]` = (1.0, 0.0) 에 놓인 기체

주소 확인: cfclient 연결 후 `radio` 로그 또는 `sys.id` 파라미터로 확인 가능

---

## 11. 파라미터 튜닝 가이드

```c
/* cbaa_formation.c 상단 #define 수정 */

// CBAA 반복 주기: 짧을수록 빠른 수렴, 너무 짧으면 패킷 충돌 증가
#define CBAA_PERIOD_MS     200   // 권장: 150~300ms

// 수렴 판정 안정 반복 수: 늘릴수록 보수적 판정
#define N_STABLE_REQUIRED  5    // 권장: 3~10

// APF 척력 게인: 크면 강한 회피, 너무 크면 불안정
#define APF_K_REP          0.05f  // 권장: 0.01~0.1

// APF 영향 반경: 드론 간격의 1.2~1.5배 권장
#define APF_RHO_0          0.6f   // 드론 간격 0.5m → 0.6~0.75m

// GoTo 지속 시간: CBAA_PERIOD_MS의 3~5배 권장 (부드러운 움직임)
#define APF_GOTO_DURATION  1.0f   // 권장: 0.6~2.0s

// 최대 반복: 안전장치
#define MAX_CBAA_ITER      100    // 20초 = 충분한 시간
```

---

## 12. 알려진 제한사항

| 항목               | 내용                                                                      | 대안                                    |
| ------------------ | ------------------------------------------------------------------------- | --------------------------------------- |
| **위치 추정**      | Flow Deck 또는 Loco Positioning 시스템 필요. 없으면 stateEstimate x/y = 0 | 외부 위치 시스템(MoCap) 연동            |
| **APF 방식**       | Receding Horizon GoTo (200ms 주기). 완전한 연속 APF 아님                  | `commanderSetSetpoint`로 속도 직접 제어 |
| **패킷 손실**      | Best-effort 브로드캐스트. N_STABLE_REQUIRED 반복으로 완화                 | DTR 프로토콜 사용 (신뢰성 향상)         |
| **z 충돌**         | APF가 xy만 처리. 모든 CF가 동일 고도로 비행 시 z 충돌 가능                | 서로 다른 TASK_POS z 할당 또는 3D APF   |
| **CF 수**          | N_AGENTS=3 하드코딩                                                       | N_AGENTS / N_TASKS 를 동적으로 설정     |
| **자기 위치 공유** | 시작 위치 하드코딩, 비행 중 위치는 stateEstimate 사용                     | UWB Loco로 절대 위치 정확도 향상        |
