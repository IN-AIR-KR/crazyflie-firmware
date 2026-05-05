# P2P 공통 헤더 필드 설명

이 문서는 [src/p2p_packets.h](src/p2p_packets.h)에 정의된 P2P 메시지의 공통 헤더 필드를 정리한 것입니다. `msg_beacon_t`, `msg_claim_t`, `msg_done_t`, `msg_snapshot_frag_t`는 모두 아래 6개 필드를 공유합니다.

## 공통 필드

| 필드 | 역할 | 이 예제에서의 의미 |
| --- | --- | --- |
| `type` | 메시지 종류 | 수신 측이 이 패킷을 어떻게 처리할지 결정합니다. `MSG_BEACON`, `MSG_CLAIM`, `MSG_DONE`, `MSG_SNAPSHOT_FR` 중 하나입니다. |
| `src_id` | 원본 송신자 ID | 메시지를 처음 만든 드론의 ID입니다. D1/D2/D3 중 하나이며, 중복 제거와 상태 추적에 사용됩니다. |
| `tx_id` | 현재 송신자 ID | 지금 이 프레임을 실제로 내보낸 노드의 ID입니다. 이 예제는 direct-only라서 보통 `src_id`와 같습니다. |
| `seq` | 시퀀스 번호 | 같은 `src_id`와 `type`의 메시지를 구분하는 번호입니다. 수신 측은 `(src, type, seq)` 조합으로 중복 패킷을 걸러냅니다. |
| `ttl` | Time To Live | 메시지가 몇 홉까지 살아 있을지 제한합니다. 현재 예제는 `TTL_MAX = 1`이라서 사실상 직접 수신만 사용합니다. |
| `hop` | 현재 홉 수 | 메시지가 지금까지 몇 번 중계되었는지 나타냅니다. 직접 송신은 `0`이고, 중계될수록 증가합니다. |

## 왜 필요한가

이 6개 필드는 "무슨 메시지인가", "누가 보냈는가", "중복인가", "멀리 퍼질 수 있는가"를 표현합니다. 즉, payload보다 먼저 봐야 하는 메타데이터입니다.

이 예제에서 특히 중요한 점은 다음입니다.

- `p2p_comm.c`는 `src_id`, `type`, `seq`를 이용해 중복 수신을 제거합니다.
- `app_main.c`는 `src_id`를 보고 어느 드론이 살아 있는지 판단합니다.
- `ttl`과 `hop`은 현재 direct-only 동작에서는 거의 단순하지만, 나중에 mesh 전파로 확장할 때 핵심이 됩니다.

## 송신 예시

`app_main.c`에서 beacon을 보낼 때의 전형적인 값은 다음과 같습니다.

```c
tx.type = MSG_BEACON;
tx.src_id = my_radio_low;
tx.tx_id = my_radio_low;
tx.seq = seq_beacon++;
tx.ttl = TTL_MAX;
tx.hop = 0u;
```

이 값들은 현재 노드가 직접 만든 1차 패킷이라는 뜻입니다. 만약 나중에 중계 기능을 붙이면 `tx_id`, `ttl`, `hop`의 의미가 더 중요해집니다.

## 한 줄 요약

`type`은 종류, `src_id`는 원본, `tx_id`는 현재 송신자, `seq`는 중복 제거용 번호, `ttl`은 전파 한도, `hop`은 누적 중계 수입니다.