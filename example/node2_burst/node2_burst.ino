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
    // 수신 FIFO에 데이터가 들어왔는지 확인
    if (p2p.available()) {
        Serial.print(F("[RX Data Received] -> Hex: "));

        // FIFO에 쌓인 모든 데이터 읽기
        while (p2p.available()) {
            uint8_t rxByte = p2p.read();

            if (rxByte < 0x10) Serial.print(F("0"));
            Serial.print(rxByte, HEX);
            Serial.print(F(" "));
        }
        Serial.println();
    }
}
