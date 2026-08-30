/*
 * -------------------------------------------------------------------------------------------------
 * node2_digital_read_burst_rx.ino - Fixed Burst Frame Sync Rx (Node 0x02)
 * -------------------------------------------------------------------------------------------------
 */

#include <SWP2P.h>


SWP2P<PRESET_W4_D4_D7> node(0x02); // Slave Node ID: 0x02
SWP2P_BIND_ISRS(PRESET_W4_D4_D7);   // 노드의 PRESET과 반드시 동일해야 함


void setup() {
    Serial.begin(115200);
    while (!Serial);

    node.begin(false); // Slave: External CLK (INT0)
    Serial.println(F("=== SWP2P Node 0x02 (Digital Read Stream Rx) Initialized ==="));
}

void loop() {
    // 1. 수신 FIFO 버퍼에 최소 1바이트 이상 들어오면 감지
    if (node.available()) {
        // [중요] 버스트 프레임의 모든 바이트가 ISR을 통해 FIFO에 완전히 들어올 때까지 미세 대기
        // (100kHz 버스 클럭 기준 4바이트 수신에 약 200~300us 소요됨)
        delayMicroseconds(500);

        uint8_t rxBuffer[SWP2P_MAX_BURST];

        // FIFO 버퍼에 쌓인 전체 패킷 바이트를 한 번에 흡수
        uint8_t receivedLen = node.readBytes(rxBuffer, SWP2P_MAX_BURST);

        Serial.print(F("[RX Burst] Received "));
        Serial.print(receivedLen);
        Serial.println(F(" Bytes:"));

        // 2. 수신받은 바이트 데이터 HEX 출력
        Serial.print(F("  -> Raw Data (HEX): "));
        for (uint8_t i = 0; i < receivedLen; i++) {
            Serial.print(F("0x"));
            if (rxBuffer[i] < 0x10) Serial.print(F("0"));
            Serial.print(rxBuffer[i], HEX);
            Serial.print(F(" "));
        }
        Serial.println();

        // 3. 패킹되어 들어온 8비트 데이터를 1비트 단위로 언패킹 출력
        Serial.print(F("  -> Stream Bit Value: "));
        for (uint8_t i = 0; i < receivedLen; i++) {
            uint8_t currentByte = rxBuffer[i];

            for (int8_t bitIdx = 7; bitIdx >= 0; bitIdx--) {
                uint8_t bitVal = (currentByte >> bitIdx) & 0x01;
                Serial.print(bitVal);
            }
            Serial.print(F(" "));
        }
        Serial.println(F("\n--------------------------------------------------"));
    }
}
