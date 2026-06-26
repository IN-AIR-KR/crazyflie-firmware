# CBBA 기반 다중 Crazyflie 분산 Task 할당 시스템

> Crazyflie 공식 펌웨어([bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware))를 fork한 레포지토리입니다.
> 핵심 코드는 `examples/app_peer_to_peer_meshprobe/` 에 위치하며, Out-of-Tree App 구조로 공식 펌웨어 위에 올라탑니다.

## 개요

3대의 Crazyflie 드론이 P2P 무선 통신으로 CBBA(Consensus-Based Bundle Algorithm)를 실행해, 사전 정의된 task 좌표들을 분산 합의 방식으로 할당하고 자율 비행하여 수행합니다.

- **CBBA**: Choi, Brunet, How (2009) 논문 기반 분산 경매 알고리즘
- **P2P Mesh**: nRF ESB 브로드캐스트 기반 직접/릴레이 통신 (선택적)
- **통신 거리 제한**: 소프트웨어 기반 통신 범위 시뮬레이션 (숨은 터미널 시나리오)
- **실시간 시각화**: Flask-SocketIO 기반 mesh_viz 웹 대시보드

## 소스 구조

```
examples/app_peer_to_peer_meshprobe/
├── Makefile              # OOT 빌드 진입점
├── app-config            # Kconfig: APP_STACKSIZE, App Layer 활성화
├── src/
│   ├── app_config.h      # 전체 실험 파라미터 (좌표, 타이밍, 통신 모드)
│   ├── app_main.c        # 앱 진입점, State Machine (IDLE→TAKEOFF→RUN→LAND→OFF)
│   ├── cbba_full.c/h     # CBBA 알고리즘 (입찰, 충돌 해결, 경로 계산)
│   ├── p2p_comm.c/h      # P2P 수신/송신, 중복 제거(seen), 이벤트 큐, 거리 필터
│   ├── p2p_packets.h     # 모든 P2P 메시지 구조체 (beacon, claim, done, bidvec)
│   └── ids.h             # 드론 ID ↔ 이름/인덱스 변환 (NODE_ID_D1=0xE6 등)
└── docs/
    ├── CBBA_OVERVIEW.md
    ├── MESH_NETWORK.md
    └── ONBOARDING.md

test_python/mesh_viz/
├── app.py                # Flask-SocketIO 시각화 서버
└── templates/index.html  # 실시간 2D 좌표 그래프 UI
```

## 빌드 및 업로드

### 사전 준비

- ARM GCC 크로스 컴파일러 (`arm-none-eabi-gcc`)
  ```bash
  # Ubuntu/Debian
  sudo apt install gcc-arm-none-eabi
  # macOS
  brew install --cask gcc-arm-embedded
  ```
- Crazyflie 2.1 기체 3대 (라디오 주소 끝 바이트: `0xE6`, `0xE7`, `0xE8`)
- Crazyradio PA USB 동글
- cflib, cfclient (`pip install cflib cfclient`)

### 1. 빌드

```bash
# 레포 루트에서
make clean
make APP=examples/app_peer_to_peer_meshprobe -j$(nproc)
```

빌드 결과물: `build/cf2.bin`

> **빌드 문제 발생 시**: `build` 폴더를 삭제 후 재시도
> ```bash
> rm -rf build
> make cf2_defconfig
> make APP=examples/app_peer_to_peer_meshprobe -j$(nproc)
> ```

### 2. 업로드

드론 1대씩 아래 과정을 반복합니다 (3대 모두 동일한 펌웨어를 업로드).

1. 드론의 전원을 끄고, **전원 버튼을 약 3초간 길게 눌러** 파란색 LED 2개가 깜빡이는 상태(부트로더 모드)로 진입
   - 배터리 부족 시 LED 1개만 깜빡임 → 충전 후 재시도
2. 명령어 실행:
   ```bash
   make cload
   ```

> **`cfloader`를 인식할 수 없다는 에러가 나오면:**
> ```bash
> pip install cflib cfclient
> ```
> 위 2개 패키지가 설치되어 있으면 `cfloader`가 포함되어 실행됩니다.
> `pip install cfloader`는 [전혀 다른 패키지](https://github.com/shachibista/cfloader)이므로 설치하면 안 됩니다.

각 드론은 자신의 라디오 주소 끝 바이트(`0xE6`/`0xE7`/`0xE8`)로 역할을 자동 구분합니다.

## 실행

### 1. 기체 전원 ON

3대를 지정된 시작 좌표(`D1_X0_M`/`D2_X0_M`/`D3_X0_M`)에 배치하고 전원을 켭니다.

`MISSION_AUTO_START=1u`이면 모든 peer가 연결된 후 `START_HOLD_MS`(3초) 대기 후 자동 이륙합니다.
`MISSION_AUTO_START=0u`이면 이륙 없이 P2P 통신만 동작합니다.

### 2. 실시간 시각화 (선택)

```bash
cd test_python/mesh_viz
pip install flask flask-socketio cflib
python3 app.py radio://0/80/2M/E7E7E7E7E7
```

브라우저에서 http://localhost:1942 접속하면 드론 위치, 통신 링크, task 마커를 실시간으로 확인할 수 있습니다.

> `app.py`의 URI 인자로 GS(Ground Station) 역할을 할 드론의 라디오 주소를 지정합니다. 해당 드론의 CRTP 포트 `0x09`를 통해 수신 비콘을 PC로 중계합니다.

## 주요 설정 (`app_config.h`)

| 항목                 | 현재값 | 설명                              |
| -------------------- | ------ | --------------------------------- |
| `MISSION_AUTO_START` | `1u`   | 1=자동 이륙+미션, 0=통신만 테스트 |
| `USE_MESH`           | `0u`   | 1=멀티홉 릴레이 활성화            |
| `USE_RANGE_LIMIT`    | `1u`   | 1=소프트웨어 통신 거리 제한       |
| `COMM_RADIUS_M`      | `0.9f` | 통신 반경 (m)                     |
| `TASK_COUNT_RUNTIME` | `3u`   | 실행할 task 수                    |
| `BUNDLE_LIMIT`       | `1u`   | 에이전트당 최대 동시 할당 task 수 |
| `TASK0~2_X/Y_M`      | -      | task 좌표 (세계 좌표계, m)        |
| `D1~3_X0/Y0_M`       | -      | 드론 시작 위치 (세계 좌표계, m)   |

## 기술 문서

[`docs/`](docs/) 디렉토리에 상세 기술 문서가 있습니다.

| 문서                                      | 내용                                                                    |
| ----------------------------------------- | ----------------------------------------------------------------------- |
| [ONBOARDING.md](docs/ONBOARDING.md)       | 프로젝트 처음 접하는 사람을 위한 전체 구조 안내, 코드 읽는 순서         |
| [CBBA_OVERVIEW.md](docs/CBBA_OVERVIEW.md) | CBBA 알고리즘 상세: bid 계산, bundle/path, 메시지 흐름, 디버깅 팁       |
| [MESH_NETWORK.md](docs/MESH_NETWORK.md)   | P2P 통신 레이어: 패킷 구조, flooding, seen 캐시, 거리 제한, 수신 흐름도 |

## 지상관제 시각화 (`test_python/mesh_viz/`)

드론이 보내는 비콘 패킷을 실시간으로 시각화하는 웹 대시보드입니다.

### 구성

```
test_python/mesh_viz/
├── app.py                # Flask-SocketIO 서버 (CRTP → WebSocket 브릿지)
└── templates/index.html  # 2D 좌표 그래프 + 패킷 로그 UI
```

### 동작 원리

1. PC에 연결된 Crazyradio를 통해 GS(Ground Station) 드론과 CRTP 링크를 맺음
2. GS 드론의 펌웨어가 수신한 P2P 비콘을 CRTP 포트 `0x09`로 PC에 중계
   - **채널 0**: 다른 드론으로부터 수신한 비콘
   - **채널 1**: GS 드론 자신이 송신한 비콘
3. `app.py`가 비콘을 파싱해 WebSocket으로 브라우저에 전달
4. `index.html`이 드론 위치, 통신 링크, task 마커를 실시간 렌더링

### 실행

```bash
cd test_python/mesh_viz
pip install flask flask-socketio cflib
python3 app.py radio://0/80/2M/E7E7E7E7E7
# 브라우저에서 http://localhost:1942 접속
```

URI 인자는 GS 역할을 할 드론의 라디오 주소입니다. 일반적으로 D2(`E7`)를 GS로 지정하면 양쪽 드론의 비콘을 모두 수신할 수 있습니다.

### UI 기능

- **2D 좌표 그래프**: 드론 위치(원), task 위치(다이아몬드), 통신 링크(화살표)를 실시간 표시
- **통신 범위 원**: 각 드론의 `COMM_RADIUS_M` 범위를 점선 원으로 표시 (토글 가능)
- **패킷 로그**: 수신/송신 비콘의 시각, 소스, 시퀀스, 좌표 기록
- **GS TX 토글**: GS 드론의 송신 화살표 및 로그 표시/숨김
- **task 토글**: task 마커 표시/숨김
- **범위 슬라이더**: 시각화용 통신 반경 조절
