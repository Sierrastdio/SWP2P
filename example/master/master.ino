#include <SWP2P.h>

// 데이터 핀 배열 (WIDTH = 4 지정)
uint8_t dataPins[] = {4, 5, 6, 7};
const uint8_t WIDTH = 4;

// 내 노드 ID: 0x01
SWP2P node(0x01);

unsigned long lastSendTime = 0;
uint8_t txCounter = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // clkIsOutput = true (D9에서 1kHz~99kHz Timer1 CTC 클럭 자동 생성 후 D2로 점퍼선 연결)
    node.begin(true, dataPins, WIDTH, 19999UL);

    Serial.println(F("=== SWP2P Node 0x01 Initialized (CLK Generator) ==="));
}

void loop() {
    // 1초마다 0x02번 노드로 카운터 값 송신
    if (millis() - lastSendTime >= 1000) {
        lastSendTime = millis();

        if (!node.isSending() && !node.isBusy()) {
            uint8_t destNode = 0x02;
            uint8_t payload = ++txCounter;

            if (node.send(destNode, payload)) {
                Serial.print(F("[TX] Sent to 0x02 -> Data: 0x"));
                Serial.println(payload, HEX);
            } else {
                Serial.println(F("[TX Error] Send failed (Busy)"));
            }
        }
    }

    // 수신 데이터 확인
    if (node.available()) {
        uint8_t rxData = node.read();
        Serial.print(F("[RX] Received -> 0x"));
        Serial.println(rxData, HEX);
    }
}
