/*
 * ----------------------------------------------------------------------------------------------------------
 * 'node1_buffer_read.ino' is an example sketch for sending shifted buffer data to 'node2_buffer_read.ino'
 * ----------------------------------------------------------------------------------------------------------
 */


#include <Arduino.h>
#include "SWP2P.h"
#include "SWP2PBuffer.h"

#define INPUT_PIN 10

SWP2P<PRESET_W4_D4_D7> p2p(0x01);
SWP2P_BIND_ISRS(PRESET_W4_D4_D7);   // must be same as node's PRESET

// make a buffer to store burst data (16 bytes)
SWP2PBuffer<16> myBuf;

uint8_t dataCounter = 0;   // Actual data to store (increments by 1 on each HIGH)
bool lastPinState = LOW;   // Previous pin state (for rising edge detection)

void setup() {
    Serial.begin(115200);

    // Set digital pin 10 as input
    pinMode(INPUT_PIN, INPUT);

    // Start as CLK Master node (50kHz)
    p2p.begin(true, 50000UL);

    Serial.println(F("[Node 1] Digital Pin 10 Input Stream Sender Initialized (5s Interval)"));
}


void loop() {
    // 1. Read digital pin 10 signal and detect rising edge (LOW -> HIGH)
    bool currentPinState = digitalRead(INPUT_PIN);

    if (currentPinState == HIGH && lastPinState == LOW) {
        // Increment value by 1 upon going HIGH
        dataCounter++;

        // Store byte in the next array slot if the buffer is not full
        if (!myBuf.full()) {
            p2p.buff(myBuf, dataCounter, true);

            Serial.print(F("[Input HIGH Detected] Added Value: 0x"));
            if (dataCounter < 0x10) Serial.print(F("0"));
            Serial.print(dataCounter, HEX);
            Serial.print(F(" | Current Buffer Length: "));
            Serial.println(myBuf.len());
        } else {
            Serial.println(F("[Warning] Buffer Full! Cannot add more data."));
        }

        delay(50); // Debounce delay
    }
    lastPinState = currentPinState;

    // 2. Periodic transmission (Every 5s, send data if present and clear the buffer)
    static unsigned long lastTxTime = 0;
    if (millis() - lastTxTime >= 5000) { // 5000ms = 5 seconds
        lastTxTime = millis();

        // Send when data exists in the buffer and the bus is idle
        if (myBuf.len() > 0 && !p2p.isSending() && !p2p.isBusy()) {
            Serial.print(F("Sending Burst Data... Length: "));
            Serial.println(myBuf.len());

            bool ok = p2p.sendBurst(0x02, myBuf);

            if (ok) {
                Serial.println(F("Burst packet queued successfully!"));
                // Clear buffer after successful transmission for next data collection
                p2p.buffFree(myBuf);
            } else {
                Serial.println(F("Failed to send burst packet."));
            }
        }
    }
}
