/*
 * -------------------------------------------------------------------------------------------------
 * node2.ino is an example sketch for receiving data and CLK from node1.ino.
 * -------------------------------------------------------------------------------------------------
 */

#include <SWP2P.h>


// you can select Data Pins and Data Bus Width.
uint8_t dataPins[] = {4, 5, 6, 7};  // if you want to use Analog Pins, you can code like this: uint8_t dataPins[] = {A0, A1, A2, A3};
const uint8_t WIDTH = 4;

// My node ID: 0x02
SWP2P node(0x02);



void setup() {

    Serial.begin(115200);
    while (!Serial);

    node.begin(false, dataPins, WIDTH);
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
