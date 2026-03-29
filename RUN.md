# P2P with PC

P2P로 드론에서 디버그 메시지를 출력하고, 이를 PC cfclient 콘솔에서 확인하는 에제입니다.

## 설정 및 실행
2ㅔ
1. `examples/app_peer_to_peer/src/peer_to_peer.c` 파일 넣기

   ```c
    while(1) {
       // Send a message every 2 seconds
       //   Note: if they are sending at the exact same time, there will be message collisions,
       //    however since they are sending every 2 seconds, and they are not started up at the same
       //    time and their internal clocks are different, there is not really something to worry about
       **DEBUG_PRINT("I am alive!\n"); // 이것만 추가!!**
       vTaskDelay(M2T(2000));
       radiolinkSendP2PPacketBroadcast(&p_reply);
     }
   ```

2. `examples/app_peer_to_peer/src/Kbuild` 내용를 아래와 같이 변경

   ```
   obj-y += peer_to_peer.o
   ```

3. `src/Kbuild` 제일 하단에 아래 내용을 추가

   ```
   obj-y += ../examples/app_peer_to_peer/src/
   ```

4. app layer 설정
   1. 설정 열기

      ```bash
      make menuconfig
      ```

   2. App    layer configuration > Enable app entry point 에서 스페스바 누르기

   3. Tab 이용해서 하단에서 Save, Exit

5. 빌드

   ```bash
   # 루트 경로에서
   make clean
   make APP=examples/app_peer_to_peer -j$(nproc)
   ```

6. 업로드
   1. 드론의 전원을 끄고, 전원 버튼을 **약 3초간 길게 눌러** 파란색 LED 2개가 깜빡이는 상태(부트로더 모드)로 만듭니다
   2. 업로드

      ```bash
      make cload
      ```

7. cfclient에서 확인하기
   1. 업로드가 완료되면 드론이 재부팅됩니다. 이제 `cfclient`를 실행하세요.
   2. **연결:** 상단의 **Scan** 버튼을 눌러 드론을 찾고 **Connect**를 클릭합니다.
   3. **콘솔 탭 활성화:** 상단 메뉴에서 **View -> Tabs -> Console**을 선택합니다. (이미 체크되어 있다면 하단 탭 목록에 'Console'이 있을 겁니다.)
   4. **메시지 확인:** 콘솔 창을 아래로 스크롤하면 2초마다 `I am alive!` 문구가 올라오는 것을 볼 수 있습니다.

## 추가 내용
- 진행에 문제가 있다면 노션 참고