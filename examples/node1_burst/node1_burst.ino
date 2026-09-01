/*
 * -------------------------------------------------------------------------------------------------
 * 'node1_burst.ino' is an example sketch for sending buffer data to 'node2_burst.ino' every 2 second.
 * -------------------------------------------------------------------------------------------------
 */


#include <Arduino.h>
#include "SWP2P.h"
#include "SWP2PBuffer.h"


SWP2P<PRESET_W4_D4_D7> p2p(0x01);
SWP2P_BIND_ISRS(PRESET_W4_D4_D7);   // must be same as node's PRESET

// create buffer to hold burst data (16 bytes)
SWP2PBuffer<16> myBuf;

void setup() {
    Serial.begin(115200);

    // Start as CLK Master node (50kHz)
    p2p.begin(true, 50000UL);

    Serial.println(F("[Node 1] SWP2P 4-Bit Burst Sender Initialized (D4~D7)"));
}

void loop() {
    static unsigned long lastTxTime = 0;

    // send burst data every 2 seconds
    if (millis() - lastTxTime >= 2000) {
        lastTxTime = millis();

        if (!p2p.isSending() && !p2p.isBusy()) {

            // 1. buffer reset
            p2p.buffFree(myBuf);

            // 2. bit input example (8bit = 1byte: 0b11001100 = 0xCC)
            p2p.buff(myBuf, 1); p2p.buff(myBuf, 1);
            p2p.buff(myBuf, 0); p2p.buff(myBuf, 0);
            p2p.buff(myBuf, 1); p2p.buff(myBuf, 1);
            p2p.buff(myBuf, 0); p2p.buff(myBuf, 0);

            // 3. byte input example
            p2p.buff(myBuf, 0xA5, true);
            p2p.buff(myBuf, 0x5A, true);
            p2p.buff(myBuf, 0xFF, true);

            // 4. clear specific position (clearSlot/clearBuffer test)
            // 0x5A at myBuf[2] is changed to 0x00, and total length(len) remains the same
            p2p.buffFree(myBuf, 2);

            Serial.print(F("Buffer ready. Total length: "));
            Serial.println(myBuf.len()); // output: 4

            // 5. send entire buffer to node 2 (0x02) via burst
            bool ok = p2p.sendBurst(0x02, myBuf);

            if (ok) {
                Serial.println(F("4-Bit Burst packet queued successfully!"));
            } else {
                Serial.println(F("Failed to send burst packet."));
            }
        }
    }
}
