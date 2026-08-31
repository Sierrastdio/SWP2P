#include <Arduino.h>
#include "SWP2P.h"
#include "SWP2PBuffer.h"

// 4비트 데이터선 규격(D4~D7) 및 Node ID 0x02 설정
SWP2P<PRESET_W4_D4_D7> p2p(0x02);
SWP2P_BIND_ISRS(PRESET_W4_D4_D7);   // must be same as node's PRESET


void setup() {
    Serial.begin(115200);

    // CLK Slave 노드로 시작
    p2p.begin(false);

    Serial.println(F("[Node 2] SWP2P 4-Bit Burst Receiver Initialized (D4~D7)"));
}

void loop() {
    // 수신 FIFO에 데이터가 들어오기 시작하면
    if (p2p.available()) {
        // 버스트 프레임 전송이 완료될 때까지 안전하게 대기 (최대 5ms)
        // BUSY 라인이 해제되거나, 더 이상 들어오는 바이트가 없을 때까지 대기
        uint8_t prevAvail = 0;
        unsigned long startWait = millis();

        while (millis() - startWait < 10) { // 최대 10ms 대기
            uint8_t currAvail = p2p.available();
            if (currAvail > 0 && currAvail == prevAvail) {
                // 데이터 수량이 더 이상 늘어나지 않고 안정화되면 대기 종료
                delayMicroseconds(500);
                if (p2p.available() == currAvail) break;
            }
            prevAvail = currAvail;
            delayMicroseconds(200);
        }

        // 전체 완성된 패킷 한 번에 읽기
        uint8_t rxBuffer[SWP2P_MAX_BURST];
        uint8_t receivedLen = p2p.readBytes(rxBuffer, SWP2P_MAX_BURST);

        if (receivedLen > 0) {
            Serial.print(F("[RX Data Received] -> Hex: "));
            for (uint8_t i = 0; i < receivedLen; i++) {
                if (rxBuffer[i] < 0x10) Serial.print(F("0"));
                Serial.print(rxBuffer[i], HEX);
                Serial.print(F(" "));
            }
            Serial.println();
        }
    }
}
