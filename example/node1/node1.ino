/*
 * -------------------------------------------------------------------------------------------------
 * node1.ino is an example sketch for CLK generating, and sending data to node2.ino every 1 second.
 * -------------------------------------------------------------------------------------------------
 */

#include <SWP2P.h>

// You can select Data Pins using DataPreset enum instead of raw arrays for maximum speed.
// Available Presets:
//   Width = 1: PRESET_W1_D4, PRESET_W1_D5, PRESET_W1_D6, PRESET_W1_D7, PRESET_W1_D9, PRESET_W1_D10, PRESET_W1_A0
//   Width = 2: PRESET_W2_D4_D5, PRESET_W2_D6_D7, PRESET_W2_D9_D10, PRESET_W2_A0_A1
//   Width = 4: PRESET_W4_D4_D7 (D4~D7), PRESET_W4_A0_A3 (A0~A3), PRESET_W4_D9_D12 (D9~D12)
//   Width = 8: PRESET_W8_A0_D7 (A0~A3 MSB + D4~D7 LSB)
const DataPreset PRESET = PRESET_W4_D4_D7;

// My node ID: 0x01
SWP2P node(0x01);

unsigned long lastSendTime = 0;
uint8_t txCounter = 0;

void setup() {

    Serial.begin(115200);
    while (!Serial);

    node.begin(true, PRESET, 36000UL);
    // 'true' means this node generates Timer1 CTC CLK in D9. false means this node receives CLK from external source (D2).
    // you have to connect D9 to D2 with jumper wire if you set 'true'.
    // with pull-up resistors(1K~4.7k), the maximum clock frequency that the example 'slave.ino' can receive is approximately 66kHz
    Serial.println(F("=== SWP2P Node 0x01 Initialized (CLK Generator) ==="));

}

void loop() {
    // every 1 second, send counter value to node 0x02
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

    // check for received data
    if (node.available()) {

        uint8_t rxData = node.read();

        Serial.print(F("[RX] Received -> 0x"));
        Serial.println(rxData, HEX);

    }
}
