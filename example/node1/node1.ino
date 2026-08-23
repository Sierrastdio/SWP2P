/*
 * -------------------------------------------------------------------------------------------------
 * node1.ino is an example sketch for CLK generating, and sending data to node2.ino every 1 second.
 * -------------------------------------------------------------------------------------------------
 */

#include <SWP2P.h>


// you can select Data Pins and Data Bus Width.
uint8_t dataPins[] = {4, 5, 6, 7};  // if you want to use Analog Pins, you can code like this: uint8_t dataPins[] = {A0, A1, A2, A3};
const uint8_t WIDTH = 4;

// My node ID: 0x01
SWP2P node(0x01);

unsigned long lastSendTime = 0;
uint8_t txCounter = 0;



void setup() {

    Serial.begin(115200);
    while (!Serial);

    node.begin(true, dataPins, WIDTH, 29900UL);
    // 'true' means this node generates Timer1 CTC CLK in D9. false means this node receives CLK from external source (D2).
    // you have to connect D9 to D2 with jumper wire if you set 'true'.
    // without pull-up resistors, the maximum clock frequency that the example 'master.ino' can transmit is approximately 76kHz
    // with pull-up resistors, the maximum clock frequency that the example 'slave.ino' can receive is approximately 29.9kHz
    // This is because the pull-up resistors make the rise time of the Pins slower,
    // which limits the maximum clock frequency, exspecially when the node is receiving data.
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
