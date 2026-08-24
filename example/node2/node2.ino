/*
 * -------------------------------------------------------------------------------------------------
 * node2.ino is an example sketch for receiving data and CLK from node1.ino.
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

// My node ID: 0x02
SWP2P node(0x02);

void setup() {

    Serial.begin(115200);
    while (!Serial);

    node.begin(false, PRESET);
    // 'false' means this node receives CLK from external source (D2). It does not generate CLK.

    Serial.println(F("=== SWP2P Node 0x02 Initialized (Receiver) ==="));

}

void loop() {

    // check for received data
    if (node.available()) {

        uint8_t rxData = node.read();

        Serial.print(F("[RX] Got Data from Bus -> 0x"));
        Serial.println(rxData, HEX);

    }
}
