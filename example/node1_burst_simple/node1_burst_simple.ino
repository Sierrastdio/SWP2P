/*
 * -------------------------------------------------------------------------------------------------
 * node1_burst_simple.ino is a simple example of SWP2P burst mode transmission (Node 0x01)
 * -------------------------------------------------------------------------------------------------
 */

#include <SWP2P.h>


SWP2P<PRESET_W4_D4_D7> p2p(0x01); // Master Node ID: 0x01
SWP2P_BIND_ISRS(PRESET_W4_D4_D7);   // must be same as node's PRESET

unsigned long lastSendTime = 0;
uint16_t packetCounter = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Start as CLK Master node (50kHz)
    p2p.begin(true, 50000UL);
    Serial.println(F("=== SWP2P Node 0x01 (Burst Tx Master) Initialized ==="));
}

void loop() {
    // 1-second (1000ms) interval transmission timer
    if (millis() - lastSendTime >= 1000) {
        lastSendTime = millis();

        // Attempt transmission only when bus is idle and not currently sending
        if (!p2p.isSending() && !p2p.isBusy()) {
            packetCounter++;

            // Prepare burst data packet to transmit (4 bytes)
            uint8_t txPayload[4];
            txPayload[0] = 0xDE;
            txPayload[1] = 0xAD;
            txPayload[2] = (uint8_t)(packetCounter >> 8);   // Packet counter High byte
            txPayload[3] = (uint8_t)(packetCounter & 0xFF); // Packet counter Low byte

            // Request 4-byte burst transmission to Node 0x02
            if (p2p.sendBurst(0x02, txPayload, 4)) {
                Serial.print(F("[TX Burst] Sent 4 Bytes to Node 0x02 -> Count: "));
                Serial.println(packetCounter);
            } else {
                Serial.println(F("[TX Burst] Send Failed (Buffer or Bus Error)"));
            }
        }
    }
}
