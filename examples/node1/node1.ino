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
bool waitingAck = false;

void setup() {

    Serial.begin(115200);
    while (!Serial);

    // Start as CLK Master node
    p2p.begin(true, 45000UL, 50000UL);
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
                waitingAck = true;

            } else {

                Serial.println(F("[TX Error] Send failed (Busy)"));

            }
        }
    }

    // check ACK result after transmission complete
    if (waitingAck && !p2p.isSending()) {

        waitingAck = false;

        if (p2p.isAckFailed()) {

            Serial.println(F("[ACK Result] ACK Failed (Timeout / No response)"));

        } else {

            Serial.println(F("[ACK Result] ACK Received Successfully!"));

        }
    }

    // check for received data
    if (p2p.available()) {

        uint8_t rxData = 0;
        uint8_t srcId = 0;

        if (p2p.read(rxData, srcId)) {

            Serial.print(F("[RX] Received from 0x"));
            if (srcId < 0x10) Serial.print(F("0"));
            Serial.print(srcId, HEX);
            Serial.print(F(" -> Data: 0x"));
            Serial.println(rxData, HEX);

        }
    }
}
