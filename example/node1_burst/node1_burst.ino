#include <Arduino.h>
#include "SWP2P.h"
#include "SWP2PBuffer.h"

// 4비트 데이터선 규격(D4~D7) 및 Node ID 0x01 설정
SWP2P<PRESET_W4_D4_D7> node(0x01);

// 버스트 데이터를 담을 버퍼 생성 (최대 16바이트)
SWP2PBuffer<16> myBuf;

void setup() {
    Serial.begin(115200);

    // CLK Master 노드로 시작 (45kHz)
    node.begin(true, 45000UL);

    Serial.println(F("[Node 1] SWP2P 4-Bit Burst Sender Initialized (D4~D7)"));
}

void loop() {
    static unsigned long lastTxTime = 0;

    // 2초마다 버스트 데이터 전송
    if (millis() - lastTxTime >= 2000) {
        lastTxTime = millis();

        if (!node.isSending() && !node.isBusy()) {

            // 1. 버퍼 초기화
            node.buffFree(myBuf);

            // 2. 비트 단위 입력 예시 (8비트 = 1바이트: 0b11001100 = 0xCC)
            node.buff(myBuf, 1); node.buff(myBuf, 1);
            node.buff(myBuf, 0); node.buff(myBuf, 0);
            node.buff(myBuf, 1); node.buff(myBuf, 1);
            node.buff(myBuf, 0); node.buff(myBuf, 0);

            // 3. 바이트 단위 입력 예시
            node.buff(myBuf, 0xA5, true);
            node.buff(myBuf, 0x5A, true);
            node.buff(myBuf, 0xFF, true);

            // 4. 특정 위치 값 0으로 지우기 (clearSlot/clearBuffer 테스트)
            // myBuf[2] 위치의 0x5A가 0x00으로 변경되며 전체 길이(len)는 유지됨
            node.buffFree(myBuf, 2);

            Serial.print(F("Buffer ready. Total length: "));
            Serial.println(myBuf.len()); // 출력: 4

            // 5. 전체 버퍼를 노드 2(0x02)로 버스트 전송
            bool ok = node.sendBurst(0x02, myBuf);

            if (ok) {
                Serial.println(F("4-Bit Burst packet queued successfully!"));
            } else {
                Serial.println(F("Failed to send burst packet."));
            }
        }
    }
}
