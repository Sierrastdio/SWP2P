#include <SWP2P.h>

// 데이터 핀 배열 (WIDTH = 4, 동일하게 맞춤)
uint8_t dataPins[] = {4, 5, 6, 7};
const uint8_t WIDTH = 4;

// 내 노드 ID: 0x02
SWP2P node(0x02);

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // clkIsOutput = false (외부 CLK 라인 수신)
    node.begin(false, dataPins, WIDTH);

    Serial.println(F("=== SWP2P Node 0x02 Initialized (Receiver) ==="));
}

void loop() {
    // 데이터 수신 검사
    if (node.available()) {
        uint8_t rxData = node.read();
        Serial.print(F("[RX] Got Data from Bus -> 0x"));
        Serial.println(rxData, HEX);
    }
}
