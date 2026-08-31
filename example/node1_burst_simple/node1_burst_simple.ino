/*
 * -------------------------------------------------------------------------------------------------
 * node1_burst.ino - SWP2P Burst Mode Tx Example (Node 0x01)
 * -------------------------------------------------------------------------------------------------
 */

#include <SWP2P.h>


SWP2P<PRESET_W4_D4_D7> node(0x01); // Master Node ID: 0x01
SWP2P_BIND_ISRS(PRESET_W4_D4_D7);   // 노드의 PRESET과 반드시 동일해야 함

unsigned long lastSendTime = 0;
uint16_t packetCounter = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Master 모드: 내부 타이머1을 사용해 50kHz 클럭 생성
    node.begin(true, 50000UL);
    Serial.println(F("=== SWP2P Node 0x01 (Burst Tx Master) Initialized ==="));
}

void loop() {
    // 1초(1000ms) 주기 전송 타이머
    if (millis() - lastSendTime >= 1000) {
        lastSendTime = millis();

        // 버스가 비어 있고 전송 중이 아닐 때만 시도
        if (!node.isSending() && !node.isBusy()) {
            packetCounter++;

            // 송신할 버스트 데이터 패킷 준비 (4바이트)
            uint8_t txPayload[4];
            txPayload[0] = 0xDE;
            txPayload[1] = 0xAD;
            txPayload[2] = (uint8_t)(packetCounter >> 8);   // 패킷 카운터 High 바이트
            txPayload[3] = (uint8_t)(packetCounter & 0xFF); // 패킷 카운터 Low 바이트

            // Node 0x02를 향해 4바이트 버스트 송신 요청
            if (node.sendBurst(0x02, txPayload, 4)) {
                Serial.print(F("[TX Burst] Sent 4 Bytes to Node 0x02 -> Count: "));
                Serial.println(packetCounter);
            } else {
                Serial.println(F("[TX Burst] Send Failed (Buffer or Bus Error)"));
            }
        }
    }
}
