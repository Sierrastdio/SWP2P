/*
 * -------------------------------------------------------------------------------------------------
 * node1.ino is an example sketch for CLK generating, and sending data to node2.ino every 1 second.
 * -------------------------------------------------------------------------------------------------
 */

#include <SWP2P.h>

const DataPreset PRESET = PRESET_W4_D4_D7;

SWP2P<PRESET_W4_D4_D7> p2p(0x01);
SWP2P_BIND_ISRS(PRESET_W4_D4_D7);   // must be same as node's PRESET


unsigned long lastSendTime = 0;
uint8_t txCounter = 0;

void setup() {

    Serial.begin(115200);
    while (!Serial);

    p2p.begin(true, 50000UL);
    // 'true' means this node generates Timer1 CTC CLK in D9. false means this node receives CLK from external source (D2).
    // you have to connect D9 to D2 with jumper wire if you set 'true'.
    Serial.println(F("=== SWP2P Node 0x01 Initialized (CLK Generator) ==="));

}

void loop() {
    // every 1 second, send counter value to p2p 0x02
    if (millis() - lastSendTime >= 1000) {

        lastSendTime = millis();

        if (!p2p.isSending() && !p2p.isBusy()) {

            uint8_t destNode = 0x02;
            uint8_t payload = ++txCounter;

            if (p2p.send(destNode, payload)) {

                Serial.print(F("[TX] Sent to 0x02 -> Data: 0x"));
                Serial.println(payload, HEX);

            } else {

                Serial.println(F("[TX Error] Send failed (Busy)"));

            }
        }
    }

    // check for received data
    if (p2p.available()) {

        uint8_t rxData = p2p.read();

        Serial.print(F("[RX] Received -> 0x"));
        Serial.println(rxData, HEX);

    }
}
