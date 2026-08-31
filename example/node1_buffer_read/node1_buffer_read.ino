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

uint8_t dataCounter = 0;   // 저장할 실제 데이터 (HIGH 될 때마다 1씩 증가)
bool lastPinState = LOW;   // 이전 핀 상태 (상승 모서리 감지용)

void setup() {
    Serial.begin(115200);

    // 디지털 10번 핀 입력 설정
    pinMode(INPUT_PIN, INPUT);

    // CLK Master 노드로 시작 (50kHz)
    p2p.begin(true, 50000UL);

    Serial.println(F("[Node 1] Digital Pin 10 Input Stream Sender Initialized (5s Interval)"));
}

void loop() {
    // 1. 디지털 10번 핀 신호 읽기 및 상승 모서리(LOW -> HIGH) 감지
    bool currentPinState = digitalRead(INPUT_PIN);

    if (currentPinState == HIGH && lastPinState == LOW) {
        // HIGH가 되는 순간 값 1 증가
        dataCounter++;

        // 버퍼가 가득 차지 않았다면 다음 배열 위치에 바이트 저장
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

        delay(50); // 디바운스 대기
    }
    lastPinState = currentPinState;

    // 2. 주기적 전송 (5초마다 데이터가 담겨있으면 전송 후 버퍼 비우기)
    static unsigned long lastTxTime = 0;
    if (millis() - lastTxTime >= 5000) { // 5000ms = 5초
        lastTxTime = millis();

        // 버퍼에 쌓인 데이터가 존재하고, 통신 선로가 비어있을 때 전송
        if (myBuf.len() > 0 && !p2p.isSending() && !p2p.isBusy()) {
            Serial.print(F("Sending Burst Data... Length: "));
            Serial.println(myBuf.len());

            bool ok = p2p.sendBurst(0x02, myBuf);

            if (ok) {
                Serial.println(F("Burst packet queued successfully!"));
                // 전송 성공 후 다음 데이터 수집을 위해 버퍼 비우기
                p2p.buffFree(myBuf);
            } else {
                Serial.println(F("Failed to send burst packet."));
            }
        }
    }
}
