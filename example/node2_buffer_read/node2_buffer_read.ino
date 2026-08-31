/*
 * -------------------------------------------------------------------------------------------------
 * 'node2_buffer_read.ino' is an example sketch for receiving shifted burst data from node1.ino.
 * -------------------------------------------------------------------------------------------------
 */


#include <Arduino.h>
#include "SWP2P.h"
#include "SWP2PBuffer.h"

// Set up 4-bit data line preset (D4~D7) and Node ID 0x02
SWP2P<PRESET_W4_D4_D7> p2p(0x02);
SWP2P_BIND_ISRS(PRESET_W4_D4_D7);   // must be same as node's PRESET


void setup() {
    Serial.begin(115200);

    // Start as CLK Slave node
    p2p.begin(false);

    Serial.println(F("[Node 2] SWP2P 4-Bit Burst Receiver Initialized (D4~D7)"));
}

void loop() {
    // When data starts arriving in the RX FIFO
    if (p2p.available()) {
        // Safely wait until burst frame transmission is complete (up to 10ms)
        // Wait until the BUSY line is released or no more incoming bytes are received
        uint8_t prevAvail = 0;
        unsigned long startWait = millis();

        while (millis() - startWait < 10) { // Wait up to 10ms
            uint8_t currAvail = p2p.available();
            if (currAvail > 0 && currAvail == prevAvail) {
                // Exit wait loop once data length stabilizes
                delayMicroseconds(500);
                if (p2p.available() == currAvail) break;
            }
            prevAvail = currAvail;
            delayMicroseconds(200);
        }

        // Read the full completed packet at once
        uint8_t rxBuffer[SWP2P_MAX_BURST];
        uint8_t receivedLen = p2p.readBytes(rxBuffer, SWP2P_MAX_BURST);

        if (receivedLen > 0) {
            Serial.print(F("[RX Data Received] -> Hex: "));
            for (uint8_t i = 0; i < receivedLen; i++) {
                if (rxBuffer[i] < 0x10) Serial.print(F("0"));
                Serial.print(rxBuffer[i], HEX);
                Serial.print(F(" "));
            }
            Serial.println();
        }
    }
}
