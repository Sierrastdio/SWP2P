/*
 * -------------------------------------------------------------------------------------------------
 * node2.ino is an example sketch for receiving data and CLK from node1.ino.
 * -------------------------------------------------------------------------------------------------
 */

#include <SWP2P.h>


SWP2P<PRESET_W4_D4_D7> node(0x02);   // node ID: 0x02

void setup() {

    Serial.begin(115200);
    while (!Serial);

    node.begin(false);
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
