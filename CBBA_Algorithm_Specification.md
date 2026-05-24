# CBBA (Consensus-Based Bundle Algorithm) 알고리즘 명세서

> 본 문서는 Choi, Brunet, How (2009) 의 *"Consensus-Based Decentralized Auctions for Robust Task Allocation"* (IEEE Transactions on Robotics, Vol. 25, No. 4) 논문을 바탕으로 작성한 구현용 명세서이다. CBAA(단일 할당)와 그 확장인 CBBA(다중 할당)의 두 단계 절차를 정확히 시뮬레이션할 수 있도록 알고리즘과 의사코드를 정리하였다.

---

## 1. 전체 전제 조건 (Preliminaries)

### 1.1 문제 정의

- **에이전트(agent)**: $N_u$ 대의 자율 차량/로봇.
  - 인덱스 집합 $\mathcal{I} = \{1, 2, \dots, N_u\}$.
- **태스크(task)**: $N_t$ 개의 작업.
  - 인덱스 집합 $\mathcal{J} = \{1, 2, \dots, N_t\}$.
- **최대 할당 수**: 각 에이전트는 최대 $L_t$ 개의 태스크를 할당받을 수 있다.
- **충돌 없는 할당(conflict-free)**: 한 태스크는 최대 1명의 에이전트에게만 할당된다.
- **목표**: 전체 reward 의 합을 최대화.
- **수렴 조건**: $N_{\min} = \min\{N_t,\ N_u L_t\}$ 개의 태스크가 모두 할당되면 종료.

### 1.2 최적화 문제 (정수계획 형태)

$$
\max \sum_{i=1}^{N_u} \sum_{j=1}^{N_t} c_{ij}(x_i, p_i)\, x_{ij}
$$

제약조건:

- $\sum_{j} x_{ij} \le L_t \quad \forall i$
- $\sum_{i} x_{ij} \le 1 \quad \forall j$
- $\sum_{i}\sum_{j} x_{ij} = N_{\min}$
- $x_{ij} \in \{0, 1\}$

여기서 $x_{ij}=1$ 이면 에이전트 $i$ 가 태스크 $j$ 를 수행. $p_i$ 는 에이전트 $i$ 의 태스크 수행 **순서 경로** (path).

### 1.3 단일 할당(Single-Assignment) vs 다중 할당(Multi-Assignment)

- **단일 할당**: $L_t = 1$ 이고 $c_{ij}(x_i, p_i) \equiv c_{ij}$ 인 특수한 경우. → **CBAA** 로 해결.
- **다중 할당**: 일반화된 경우. 한 에이전트가 여러 태스크를 순서를 가진 경로로 수행. → **CBBA** 로 해결.

### 1.4 점수 함수(Scoring Function) 가정

- $c_{ij}(x_i, p_i) \ge 0$ (비음수).
- 다중 할당의 경우 **DMG (Diminishing Marginal Gain)** 가정:
  $$ c_{ij}[b_i] \ge c_{ij}[b_i \oplus_{\text{end}} b] $$
  즉, 번들에 다른 태스크를 더 추가한 뒤에 task $j$ 를 추가했을 때의 한계점수는, 그 전 시점에 추가했을 때의 한계점수보다 크지 않다.
- 본 논문에서 사용한 예시 점수 함수 — **시간 할인 보상(Time-Discounted Reward)**:
  $$ S_i^{p_i} = \sum_{j} \lambda_j^{\tau_i^j(p_i)} \bar{c}_j $$
  - $\lambda_j < 1$ : 태스크 $j$ 의 할인 인자.
  - $\tau_i^j(p_i)$ : 경로 $p_i$ 를 따라 에이전트 $i$ 가 태스크 $j$ 위치에 도착하는 데 걸리는 시간.
  - $\bar{c}_j$ : 태스크 $j$ 의 정적 점수.

### 1.5 통신 네트워크

- 시각 $\tau$ 의 통신 그래프 $G(\tau)$ 는 무방향 그래프.
- 인접행렬 $g_{ik}(\tau) = 1$ : 에이전트 $i, k$ 가 이웃.
- 관례: $g_{ii}(\tau) = 1$ (자기 자신은 항상 연결).
- 네트워크 직경(diameter) $D = \max_{(i,k)} d_{ik}$.

### 1.6 동률 처리 (Tie-Breaking)

- Phase 1 의 $J_i$ 결정 또는 Phase 2 의 $z_{i,J_i}$ 결정 시 동률(tie)이 발생할 수 있다.
- **체계적 방법**으로 해결해야 한다. 예: 에이전트 ID와 태스크 ID 의 사전식(lexicographical) 순서.

---

## 2. CBAA — Consensus-Based Auction Algorithm (단일 할당)

CBAA 는 $L_t = 1$ 인 단일 할당 문제를 위한 알고리즘으로, **Phase 1 (Auction)** 과 **Phase 2 (Consensus)** 의 두 단계를 반복(iteration)한다.

### 2.1 각 에이전트가 보유하는 자료구조

| 변수 | 차원 | 설명 |
|---|---|---|
| $x_i$ | $\{0,1\}^{N_t}$ | 태스크 할당 벡터. $x_{ij}=1$ 이면 task $j$ 가 자기 자신에게 할당. |
| $y_i$ | $\mathbb{R}_+^{N_t}$ | winning bid list. 각 태스크에 대한 현재까지 알려진 최고 입찰가의 추정값. |
| $c_{ij}$ | $\mathbb{R}_+$ | 에이전트 $i$ 가 태스크 $j$ 에 매기는 점수(입찰가). |
| $h_i$ | $\{0,1\}^{N_t}$ | 유효 태스크 리스트 (valid task list). |

**초기화**: $x_i = \mathbf{0}, \quad y_i = \mathbf{0}$.

### 2.2 Phase 1: Auction Process

**핵심 아이디어**: 아직 태스크를 할당받지 못한 에이전트는, 현재 자신이 아는 winning bid 보다 자신이 더 높게 입찰할 수 있는 태스크들 중에서 자신에게 가장 점수가 높은 태스크를 선택한다.

**유효 태스크 판정**:
$$ h_{ij} = \mathbb{I}(c_{ij} > y_{ij}) \quad \forall j \in \mathcal{J} $$
여기서 $\mathbb{I}(\cdot)$ 는 지시함수(인자가 참이면 1, 거짓이면 0).

#### Pseudo-code: CBAA Phase 1 (Algorithm 1)

```
procedure SELECT_TASK(c_i, x_i(t-1), y_i(t-1)):
    1:  x_i(t) ← x_i(t-1)
    2:  y_i(t) ← y_i(t-1)
    3:  if sum_j x_ij(t) == 0 then          # 아직 할당받지 못한 에이전트만
    4:      for each j in J:
    5:          h_ij ← 1 if c_ij > y_ij(t) else 0
    6:      if h_i != 0 then                 # 유효 태스크가 하나라도 있으면
    7:          J_i ← argmax_j (h_ij * c_ij)  # 동률은 체계적으로 해결
    8:          x_{i, J_i}(t) ← 1
    9:          y_{i, J_i}(t) ← c_{i, J_i}
    10:     end if
    11: end if
end procedure
```

### 2.3 Phase 2: Consensus Process

**핵심 아이디어**: 모든 이웃과 winning bid 정보를 교환하고, **max-consensus** 를 통해 각 태스크의 최고 입찰가에 수렴한다. 자신이 outbid 당했다면 (이웃이 더 높은 값을 가지고 있다면) 자신의 할당을 포기한다.

**업데이트 규칙**:
- 각 태스크 $j$ 에 대해: $y_{ij}(t) = \max_k\ g_{ik}(\tau) \cdot y_{kj}(t)$
- 자기 자신이 선택한 태스크 $J_i$ 에 대해 winner 가 자신이 아니면 → 할당 해제 ($x_{i,J_i} = 0$).

#### Pseudo-code: CBAA Phase 2 (Algorithm 2)

```
1:  SEND y_i to all k with g_ik(τ) = 1
2:  RECEIVE y_k from all k with g_ik(τ) = 1

procedure UPDATE_TASK(g_i(τ), {y_k for k where g_ik=1}, J_i):
    3:  for each j in J:
    4:      y_ij(t) ← max_k (g_ik(τ) * y_kj(t))
    5:  z_{i, J_i} ← argmax_k (g_ik(τ) * y_{k, J_i}(t))
    6:  if z_{i, J_i} != i then               # 자신이 outbid 당했으면
    7:      x_{i, J_i}(t) ← 0                 # 할당 해제
    8:  end if
end procedure
```

### 2.4 CBAA 반복 종료 조건

- 모든 에이전트가 태스크를 할당받았거나 ($\sum_j x_{ij}=1$),
- 또는 더 이상 유효 태스크가 없을 때 ($h_i = 0$) 까지 Phase 1 / Phase 2 를 반복.
- 정적 네트워크 + 직경 $D$ 에서 최대 $N_{\min} \cdot D$ iteration 내에 수렴 보장.

---

## 3. CBBA — Consensus-Based Bundle Algorithm (다중 할당)

CBBA 는 CBAA 를 일반화한 알고리즘으로, 각 에이전트가 **번들(bundle)** 단위로 여러 개의 태스크를 할당받을 수 있다. 핵심 차이는 다음과 같다:

- 입찰은 여전히 **태스크 단위**로 이루어진다 (기존 combinatorial auction 처럼 번들 전체에 입찰하지 않음).
- 각 에이전트는 단일 번들을 유지하고, 한계 점수(marginal score)가 가장 큰 태스크를 순차적으로 추가한다.
- 충돌 해결 시 더 복잡한 규칙이 필요하다 (timestamp 기반).

### 3.1 각 에이전트가 보유하는 자료구조

| 변수 | 차원 | 설명 |
|---|---|---|
| $y_i$ | $\mathbb{R}_+^{N_t}$ | winning bid list. 각 태스크의 현재 최고 입찰가. |
| $z_i$ | $\mathcal{I}^{N_t}$ | winning agent list. 각 태스크의 현재 winner ID (없으면 $\emptyset$). |
| $b_i$ | $(\mathcal{J} \cup \{\emptyset\})^{L_t}$ | **번들** — 태스크를 추가한 시간 순서로 정렬된 리스트. |
| $p_i$ | $(\mathcal{J} \cup \{\emptyset\})^{L_t}$ | **경로** — 태스크를 실제 수행할 위치(순서)대로 정렬된 리스트. |
| $s_i$ | $\mathbb{R}^{N_u}$ | timestamp 벡터. 다른 에이전트로부터 마지막 정보 갱신 시각. |

> **번들(b) vs 경로(p) 의 구분**: 번들은 "언제 추가되었는가"(시간) 의 순서, 경로는 "어디서 수행되는가"(공간/순서) 의 순서. 새 태스크를 추가할 때는 경로상 가장 점수 증가가 큰 위치에 삽입한다.

### 3.2 한계 점수(Marginal Score) 정의

$$
c_{ij}[b_i] =
\begin{cases}
0, & \text{if } j \in b_i \\
\max_{n \le |p_i|} S_i^{p_i \oplus_n \{j\}} - S_i^{p_i}, & \text{otherwise}
\end{cases}
$$

여기서 $\oplus_n$ 은 첫 번째 리스트의 $n$ 번째 원소 바로 뒤에 두 번째 리스트를 삽입하는 연산이다. 즉, 새 태스크 $j$ 를 경로 $p_i$ 의 모든 가능한 위치에 삽입해보고, **가장 큰 점수 증가를 주는 위치**를 선택한다.

### 3.3 번들 / 경로 업데이트 규칙

새 태스크 $J_i$ 를 결정하고 번들·경로에 삽입:

- $J_i = \arg\max_j\ (c_{ij}[b_i] \times h_{ij})$, 단 $h_{ij} = \mathbb{I}(c_{ij} > y_{ij})$
- $n_{i,J_i} = \arg\max_n\ S_i^{p_i \oplus_n \{J_i\}}$ — 경로상 최적 삽입 위치
- 번들 업데이트: $b_i \leftarrow b_i \oplus_{\text{end}} \{J_i\}$ (번들 끝에 추가)
- 경로 업데이트: $p_i \leftarrow p_i \oplus_{n_{i,J_i}} \{J_i\}$ (경로상 최적 위치에 삽입)

### 3.4 Phase 1: Bundle Construction

**핵심 아이디어**: 번들 크기가 $L_t$ 에 도달하거나 더 이상 유효한 태스크가 없을 때까지, 한계 점수가 가장 큰 태스크를 번들에 계속 추가한다.

#### Pseudo-code: CBBA Phase 1 (Algorithm 3)

```
procedure BUILD_BUNDLE(z_i(t-1), y_i(t-1), b_i(t-1)):
    1:  y_i(t) ← y_i(t-1)
    2:  z_i(t) ← z_i(t-1)
    3:  b_i(t) ← b_i(t-1)
    4:  p_i(t) ← p_i(t-1)
    5:  while |b_i| < L_t do
    6:      for each j in J \ b_i:                            # 번들에 없는 태스크에 대해
    7:          c_ij ← max_{n ≤ |p_i|} S_i^{p_i ⊕_n {j}} - S_i^{p_i}
    8:      for each j in J:
    9:          h_ij ← 1 if c_ij > y_ij else 0
    10:     J_i ← argmax_j (c_ij * h_ij)
    11:     if no valid task (모든 h_ij = 0 or 모든 c_ij*h_ij = 0) then
    12:         break
    13:     n_{i, J_i} ← argmax_n S_i^{p_i ⊕_n {J_i}}        # 최적 삽입 위치
    14:     b_i ← b_i ⊕_end {J_i}                              # 번들 끝에 추가
    15:     p_i ← p_i ⊕_{n_{i,J_i}} {J_i}                      # 경로 최적 위치에 삽입
    16:     y_{i, J_i}(t) ← c_{i, J_i}
    17:     z_{i, J_i}(t) ← i
    18: end while
end procedure
```

> **주의**: CBAA 의 $x_i$ 와 달리 CBBA 에서는 $z_i$ (winning agent list) 를 사용한다. 단순히 "내가 outbid 되었나?" 만이 아니라 "누가 winner 인가?" 정보까지 필요한데, 이는 더 정교한 충돌 해결 규칙(아래 Table 1) 때문이다.

### 3.5 timestamp 벡터 $s_i$ 갱신 규칙

각 메시지가 수신될 때:

$$
s_{ik} =
\begin{cases}
\tau_r, & \text{if } g_{ik}=1 \quad \text{(직접 통신한 경우)} \\
\max_{m:\, g_{im}=1}\ s_{mk}, & \text{otherwise} \quad \text{(이웃들의 정보 중 최신값)}
\end{cases}
$$

여기서 $\tau_r$ 은 메시지 수신 시각.

### 3.6 Phase 2: Conflict Resolution — 수신 시 처리

에이전트 $i$ 가 이웃 $k$ 로부터 메시지 $(y_k, z_k, s_k)$ 를 수신했을 때, **각 태스크 $j$ 에 대해** 다음 세 가지 동작 중 하나를 수행한다:

| Action | 의미 | 동작 |
|---|---|---|
| **update** | sender 의 정보로 덮어쓴다 | $y_{ij} \leftarrow y_{kj},\ z_{ij} \leftarrow z_{kj}$ |
| **reset** | 정보 초기화 | $y_{ij} \leftarrow 0,\ z_{ij} \leftarrow \emptyset$ |
| **leave** | 변경 없음 | $y_{ij} \leftarrow y_{ij},\ z_{ij} \leftarrow z_{ij}$ |

#### Table 1 — 충돌 해결 행동 규칙 (전체)

> **읽는 법**: 각 행은 "sender $k$ 가 생각하는 winner ($z_{kj}$)" 와 "receiver $i$ 가 생각하는 winner ($z_{ij}$)" 의 조합. 마지막 열이 receiver 의 동작. **기본값은 leave**.

##### Case A: Sender $k$ 가 winner 는 자기 자신($k$)이라고 생각 ($z_{kj} = k$)

| Receiver $i$ 가 생각하는 winner ($z_{ij}$) | 조건 | Receiver 의 동작 |
|---|---|---|
| $i$ (자기 자신) | $y_{kj} > y_{ij}$ 이면 | **update** |
| $k$ (sender 와 같음) | 항상 | **update** |
| $m \notin \{i, k\}$ (제3자) | $s_{km} > s_{im}$ **또는** $y_{kj} > y_{ij}$ 이면 | **update** |
| none (아무도 없음) | 항상 | **update** |

##### Case B: Sender $k$ 가 winner 는 receiver $i$ 라고 생각 ($z_{kj} = i$)

| Receiver $i$ 가 생각하는 winner ($z_{ij}$) | 조건 | Receiver 의 동작 |
|---|---|---|
| $i$ (자기 자신) | 항상 | **leave** |
| $k$ (sender) | 항상 | **reset** |
| $m \notin \{i, k\}$ (제3자) | $s_{km} > s_{im}$ 이면 | **reset** |
| none | 항상 | **leave** |

##### Case C: Sender $k$ 가 winner 는 제3자 $m \notin \{i, k\}$ 라고 생각 ($z_{kj} = m$)

| Receiver $i$ 가 생각하는 winner ($z_{ij}$) | 조건 | Receiver 의 동작 |
|---|---|---|
| $i$ (자기 자신) | $s_{km} > s_{im}$ **그리고** $y_{kj} > y_{ij}$ 이면 | **update** |
| $k$ (sender) | $s_{km} > s_{im}$ 이면 | **update** |
| $k$ (sender) | else (위 조건이 아니면) | **reset** |
| $m$ (제3자 본인) | $s_{km} > s_{im}$ 이면 | **update** |
| $n \notin \{i, k, m\}$ (또 다른 제3자) | $s_{km} > s_{im}$ **그리고** $s_{kn} > s_{in}$ 이면 | **update** |
| $n \notin \{i, k, m\}$ | $s_{km} > s_{im}$ **그리고** $y_{kj} > y_{ij}$ 이면 | **update** |
| $n \notin \{i, k, m\}$ | $s_{kn} > s_{in}$ **그리고** $s_{im} > s_{km}$ 이면 | **reset** |
| none | $s_{km} > s_{im}$ 이면 | **update** |

##### Case D: Sender $k$ 가 winner 가 없다고 생각 ($z_{kj} = \emptyset$)

| Receiver $i$ 가 생각하는 winner ($z_{ij}$) | 조건 | Receiver 의 동작 |
|---|---|---|
| $i$ (자기 자신) | 항상 | **leave** |
| $k$ (sender) | 항상 | **update** |
| $m \notin \{i, k\}$ (제3자) | $s_{km} > s_{im}$ 이면 | **update** |
| none | 항상 | **leave** |

> **표 해석 요지**:
> - "더 최신 정보를 가진 쪽" (timestamp $s$ 가 큰 쪽) 의 의견이 우선시된다.
> - 직접적인 상호 동의 (둘 다 같은 winner 를 지목) 는 그대로 신뢰한다.
> - 정보가 충돌하지만 누가 맞는지 모를 때는 **reset** 으로 안전하게 초기화한다.
> - sender 와 receiver 가 직접 관련된 경우 (자기 자신이 winner) 는 timestamp 비교 없이 즉시 결정한다.

### 3.7 번들 / 경로 재구성 (Bundle Release)

Table 1 의 규칙에 따라 어떤 태스크 $j$ 의 winning bid 가 **update** 또는 **reset** 되었을 때, 만약 그 태스크가 자신의 번들 안에 있었다면, **그 태스크와 그 이후에 추가된 모든 태스크를 번들에서 제거**해야 한다.

이유: 번들 안에서 한 태스크가 빠지면, 그 뒤에 한계 점수로 계산하여 추가했던 다른 태스크들의 점수가 더 이상 유효하지 않게 되기 때문이다.

$$
\bar{n}_i = \min\{n : z_{i, b_{in}} \ne i\}
$$

- 모든 $n \ge \bar{n}_i$ 에 대해 $b_{in} \leftarrow \emptyset$ (해당 위치 이후 모두 제거).
- 모든 $n > \bar{n}_i$ 에 대해 $y_{i, b_{in}} \leftarrow 0,\ z_{i, b_{in}} \leftarrow \emptyset$ (그 이후 항목들의 winning bid/agent 도 초기화).
- 경로 $p_i$ 에서도 해당 태스크들을 제거 (경로 순서를 유지하며).

> **중요**: $b_{i, \bar{n}_i}$ 자신은 이미 누가 가져갔으므로 $y$, $z$ 를 reset 하지 않는다 (이미 다른 에이전트의 정보가 들어 있어야 정상). 그 뒤 ($n > \bar{n}_i$) 의 항목들만 reset 한다.

### 3.8 CBBA 알고리즘 전체 흐름

```
초기화:
    각 에이전트 i에 대해:
        y_i ← 0, z_i ← {∅, ..., ∅}, b_i ← {}, p_i ← {}, s_i ← 0

반복 (수렴할 때까지):
    각 에이전트 i에 대해:
        # Phase 1: Bundle Construction
        BUILD_BUNDLE(z_i, y_i, b_i)
        
        # Communication
        SEND (y_i, z_i, s_i) to all neighbors k where g_ik = 1
        RECEIVE (y_k, z_k, s_k) from all neighbors
        
        # Phase 2: Conflict Resolution
        for each neighbor k where g_ik = 1:
            for each task j in J:
                action ← lookup Table 1 with (z_kj, z_ij, conditions on s, y)
                if action == update:
                    y_ij ← y_kj
                    z_ij ← z_kj
                elif action == reset:
                    y_ij ← 0
                    z_ij ← ∅
            update s_i using s_k according to (3.5)
        
        # Bundle Release (위 동작으로 인해 변경된 태스크가 번들에 있으면 정리)
        if any updated/reset task j is in b_i:
            n̄ ← min n such that z_{i, b_in} ≠ i
            번들에서 인덱스 ≥ n̄ 의 항목 제거
            경로에서도 해당 태스크들 제거
            y, z 도 (3.7) 규칙에 따라 정리

종료 조건:
    모든 에이전트의 (y_i, z_i, b_i) 가 더 이상 변하지 않고 충돌이 없을 때.
```

---

## 4. 수렴 및 성능 보장

### 4.1 수렴 (Convergence)

- **정적 네트워크 + 직경 $D$ + 동기 통신 + DMG 점수**: 최대 $N_{\min} \cdot D$ iteration 내에 충돌 없는 할당으로 수렴.
- **DMG 가 아닌 경우**: 다음과 같이 점수를 보정하면 수렴 보장.
  $$ c_{ij}(t) = \min\{ c^o_{ij}(t),\ c_{ij}(t-1) \} $$
  (단조 감소를 강제)
- **동적 네트워크 + 비동기 통신**: 어떤 유한한 $\rho < \infty$ 동안의 그래프 합집합이 연결되어 있으면 수렴.
- **SA(Situational Awareness) 불일치**: 수렴 자체에는 영향 없음 (단, 성능은 저하 가능).

### 4.2 성능 (Performance)

- CBBA 의 해는 중앙집중식 Sequential Greedy Algorithm (SGA) 과 동일하다.
- **50% 최적성 보장**: $\text{MOPT} \le 2 \cdot \text{CBBA}$. (단, SA 가 정확하고 점수가 DMG 일 때)

---

## 5. 구현 시 체크리스트

다음 항목을 모두 구현해야 CBBA 시뮬레이션이 완성된다:

1. **데이터 구조**: 각 에이전트의 $y_i, z_i, b_i, p_i, s_i$ 와 통신 그래프 $G$.
2. **점수 함수**: 시간 할인 보상 (또는 다른 DMG 점수). 한계 점수 $c_{ij}[b_i]$ 와 경로상 최적 삽입 위치 계산 함수.
3. **Phase 1 (Bundle Construction)**: 위 Algorithm 3.
4. **통신 모델**: 동기/비동기 선택, 통신 그래프의 정적/동적 여부.
5. **Phase 2 (Conflict Resolution)**: Table 1 의 24 개 규칙을 모두 정확히 구현. 디폴트는 leave.
6. **번들 해제(Bundle Release)**: 한 태스크가 update/reset 되면 그 이후 번들 항목 모두 제거.
7. **Timestamp 갱신**: 직접 통신/간접 통신 두 경우 모두 처리.
8. **동률 처리**: 사전식(lexicographical) 등 체계적 방법.
9. **수렴 판정**: 모든 에이전트의 상태가 더 이상 변하지 않고, 모든 제약조건이 만족되는지 확인.
10. **검증용 지표**: 총 점수, 수렴 시간 (iteration 수), 충돌 여부, 50% 최적성 등.

---

## 참고 문헌

Choi, H.-L., Brunet, L., & How, J. P. (2009). *Consensus-Based Decentralized Auctions for Robust Task Allocation*. IEEE Transactions on Robotics, 25(4), 912–926.
