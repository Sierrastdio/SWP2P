/**********************************************************************************************************
 * SWP2P
 * Copyright (c) 2026 Sierrastdio
 *
 * SWP2P is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SWP2P is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 *            <http://www.gnu.org/licenses/>.
 *********************************************************************************************************/

#ifndef SWP2P_H
#define SWP2P_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "SWP2PBuffer.h"

#define SWP2P_FIFO_DEPTH 16
#define SWP2P_MAX_BURST   16  // _txBuffer 크기. len-2를 8비트 레지스터에 실어 보내므로 이론상 257까지 가능하지만
                              // RAM 절약 위해 우선 16으로 제한 (필요시 늘리면 됨).
                              // WARNING: 이거 때문에 SWP2PBuffer<32> myBuf; 와 같이 16바이트를 초과하는 버퍼를 설정할 경우,
                              //          자동 분할 전송되지 않으며 버퍼 오버플로우가 발생할 수 있음.
                              //          16바이트씩 두번의 통신 프레임으로 전송될것임.
                              //          16바이트 초과 데이터 전송이 필요하면 이 상수를 해당 크기만큼 늘려야 함.
                              //          아니면 myBuf1, myBuf2 등 한 데이터에 여러개의 버퍼를 사용해야함.

// ---- 주소 인코딩 ----
// destId 바이트의 MSB(bit7)를 "burst 프레임 여부" 플래그로 사용.
// 그 결과 실제 NODE_ID는 0~126(7비트)만 쓸 수 있고, 브로드캐스트 주소도 0x7F로 바뀐다.
// (노드 수가 10개 안팎이라 7비트 주소공간으로 충분하다는 전제)
#define SWP2P_NODE_MASK   0x7F  // 0111 1111
#define SWP2P_BURST_BIT   0x80  // 1000 0000
#define SWP2P_BROADCAST   0x7F  // 0111 1111

#define FLAG_IS_SENDING   0
#define FLAG_IS_BUSY      1
#define FLAG_RX_CAPTURED  2
#define FLAG_IS_MY_PACKET 3
#define FLAG_RX_IS_BURST  4   // 이번 수신 프레임이 burst인지 (RX_SRC 완료 시점에 결정되어 저장됨)
#define FLAG_ACK_FAILED   5   // 직전 송신이 ACK를 못 받고 타임아웃됐는지 (isAckFailed()로 조회)

// GPIOR0 플래그 비트 연산 추적 주석 추가
#define SET_FLAG(b) (GPIOR0 |= (1 << (b)))   // [xxxx xxxx] |= [0000 0001 << b]  => [b번 비트만 1로 Set]
#define CLR_FLAG(b) (GPIOR0 &= ~(1 << (b)))  // [xxxx xxxx] &= [1111 1110 << b]  => [b번 비트만 0으로 Clear]
#define GET_FLAG(b) (GPIOR0 & (1 << (b)))   // [xxxx xxxx] &  [0000 0001 << b]  => [b번 비트가 1이면 >0, 0이면 0]

enum DataPreset : uint8_t {
    PRESET_W1_D4 = 0, PRESET_W1_D5, PRESET_W1_D6, PRESET_W1_D7, PRESET_W1_D9, PRESET_W1_D10, PRESET_W1_A0,
    PRESET_W2_D4_D5, PRESET_W2_D6_D7, PRESET_W2_D9_D10, PRESET_W2_A0_A1,
    PRESET_W4_D4_D7, PRESET_W4_A0_A3, PRESET_W4_D9_D12,
    PRESET_W8_A0_D7
};

/* Arduino pinout
 *
 * D0~D7:   PD0~PD7
 * D8~D13:  PB0~PB5
 * A0~A5:   PC0~PC5
 *
 */

// ---- Tx/Rx 상태 번호 ----
// Tx: 0 IDLE, 1 PENDING, 2 ARB, 3 LEN, 4 DATA, 5 RELEASE/DONE, 6 LOST
// Rx: 0 IDLE, 1 ADDR, 2 SRC, 3 LEN, 4 DATA, 5 ACK

class SWP2PBase {
public:
    static uint8_t _nodeId;
    static volatile uint8_t _rxFifo[SWP2P_FIFO_DEPTH];
    static volatile uint8_t _rxSrcFifo[SWP2P_FIFO_DEPTH]; // [발신자 ID 수정] _rxFifo와 같은 인덱스로 동기화되는 발신자 ID 큐
    static volatile uint8_t _rxHead;
    static volatile uint8_t _rxTail;
    static volatile uint8_t _rxCount;

    // _txState/_rxState는 SRAM 대신 GPIOR1/GPIOR2에 직접 상주시킨다.
    #define _txState GPIOR1
    #define _rxState GPIOR2

    static uint8_t _txDestId;      // MSB=burst 플래그가 이미 인코딩된 상태로 저장됨
    static uint8_t _txData;        // len==1일 때만 사용 (기존 빠른 경로 그대로 유지)
    static uint8_t _txBuffer[SWP2P_MAX_BURST]; // len>1일 때 사용
    static uint8_t _txLen;
    static uint8_t _txIdx;
    static volatile uint16_t _txArbReg;
    static volatile uint8_t _txDataReg;   // 데이터 바이트 전송용 + LEN 필드 전송에도 재사용
    static volatile uint8_t _arbChunkCount;
    static volatile uint8_t _txDataChunkCount;
    static volatile uint8_t _arbMyChunk;
    static volatile uint8_t _arbChunksSent; // [버그1 수정] 이번 중재 라운드에서 "드라이브+비교까지 끝난" 청크 수(이번 것 포함)
    static volatile uint8_t _ackWaitCount;  // [버그4 수정] ACK 대기 중 남은 edge 수 (0이면 타임아웃)

    static volatile uint8_t _rxAddrByte;
    static volatile uint8_t _rxSrcByte;
    static volatile uint8_t _rxDataByte;   // 데이터 바이트 수신용 + LEN 필드 수신에도 재사용
    static volatile uint8_t _rxChunkCount;
    static volatile uint8_t _rxLen;        // 이번 프레임에서 받아야 할 총 바이트 수
    static volatile uint8_t _rxByteIdx;    // 지금까지 받은 바이트 수

    static void _fifoPush(uint8_t val, uint8_t srcId); // [발신자 ID 수정]
    static void setupTimer1(unsigned long freq);
    static void stopTimer1();
};

template <DataPreset PRESET>
class SWP2P : public SWP2PBase {
public:
    static constexpr uint8_t BROADCAST = SWP2P_BROADCAST;

    SWP2P(uint8_t nodeId)
    {
        // [xxxx xxxx] & [0111 1111] => [0xxx xxxx] (7비트 주소만 마스킹)
        _nodeId = nodeId & SWP2P_NODE_MASK;
    }

    void begin(bool clkIsOutput, unsigned long clkFreq = 100000UL)
    {
        GPIOR0 = 0;
        _txState = 0; // GPIOR1
        _rxState = 0; // GPIOR2

        // DDRD2(0b00000100) 비트 0으로 클리어 (D2 입력 설정)
        DDRD &= ~(1 << DDD2);   // [xxxx xxxx] &= [1111 1011] => [xxxx x0xx]
        // PORTD2(0b00000100) 비트 0으로 클리어 (D2 풀업 해제)
        PORTD &= ~(1 << PORTD2); // [xxxx xxxx] &= [1111 1011] => [xxxx x0xx]

        PORTD &= ~(1 << PORTD3); // [xxxx xxxx] &= [1111 0111] => [xxxx 0xxx] (D3 출력 0)
        DDRD &= ~(1 << DDD3);   // [xxxx xxxx] &= [1111 0111] => [xxxx 0xxx] (D3 입력 설정)
        PORTB &= ~(1 << PORTB0); // [xxxx xxxx] &= [1111 1110] => [xxxx xxx0] (PB0 출력 0)
        DDRB &= ~(1 << DDB0);   // [xxxx xxxx] &= [1111 1110] => [xxxx xxx0] (PB0 입력 설정)

        _dataRelease();

        if (clkIsOutput) {
            DDRB |= (1 << DDB1); // [xxxx xxxx] |= [0000 0010] => [xxxx xx1x] (PB1 출력 설정)
            setupTimer1(clkFreq);
        } else {
            TCCR1A = 0;
            TCCR1B = (1 << CS10); // [0000 0000] = [0000 0001] (Timer1 분주비 1 설정)
        }

        // TCCR1B: 노이즈 캔슬러(ICNC1) 및 엣지 세팅
        TCCR1B |= (1 << ICNC1); // [xxxx xxxx] |= [1000 0000] => [1xxx xxxx] (노이즈 캔슬러 ON)
        TCCR1B |= (1 << ICES1); // [xxxx xxxx] |= [0100 0000] => [x1xx xxxx] (Rising Edge 캡처)
        TIMSK1 |= (1 << ICIE1); // [xxxx xxxx] |= [0010 0000] => [xx1x xxxx] (Capture 인터럽트 ON)

        // EICRA: INT0 설정 (ISC01, ISC00 비트)
        EICRA &= ~((1 << ISC01) | (1 << ISC00)); // [xxxx xxxx] &= [1111 0011] => [xxxx 00xx] (INT0 초기화)
        EICRA |= (1 << ISC00);                   // [xxxx xxxx] |= [0000 0001] => [xxxx xx01] (INT0 Logical Change)
        EIMSK |= (1 << INT0);                    // [xxxx xxxx] |= [0000 0001] => [xxxx xxx1] (INT0 Enable)

        // EICRA: INT1 설정 (ISC11, ISC10 비트)
        // 주의: Low Level(00)로 두면 D3(BUSY)가 LOW인 "동안"만 인터럽트가 걸리고, HIGH로
        // 복귀하는 상승 엣지(=버스 idle 전환)를 감지할 방법이 없다. _onBusyEdge()의
        // idle 분기(FLAG_IS_BUSY를 푸는 유일한 지점)가 바로 그 상승 엣지에서 실행돼야
        // 하므로, Any Logical Change(01)로 바꿔서 양쪽 엣지를 모두 잡아야 한다.
        // (이게 없으면 최초 1회 통신 후 FLAG_IS_BUSY가 영구히 안 풀림 -> 이후 전송 전부 조용히 무시됨)
        EICRA &= ~((1 << ISC11) | (1 << ISC10)); // [xxxx xxxx] &= [1111 1100] => [xxxx xxxx] (INT1 초기화)
        EICRA |= (1 << ISC10);                   // [xxxx xxxx] |= [0000 0100] => [xxxx x1xx] (INT1 Any Logical Change)
        EIMSK |= (1 << INT1);                    // [xxxx xxxx] |= [0000 0010] => [xxxx xx1x] (INT1 Enable)

        sei();
    }

    bool send(uint8_t destId, uint8_t data)
    {
        return sendBurst(destId, &data, 1);
    }

    bool sendBurst(uint8_t destId, const uint8_t* buf, uint8_t len)
    {
        if (len == 0 || len > SWP2P_MAX_BURST) return false;

        if (GET_FLAG(FLAG_IS_SENDING) || GET_FLAG(FLAG_IS_BUSY)) return false;

        uint8_t d = destId & SWP2P_NODE_MASK; // [xxxx xxxx] & [0111 1111] => [0xxx xxxx]

        if (len > 1) d |= SWP2P_BURST_BIT;     // [0xxx xxxx] |= [1000 0000] => [1xxx xxxx] (Burst 비트 Set)
        _txDestId = d;
        _txLen = len;
        _txIdx = 0;

        if (len == 1) {
            _txData = buf[0];
        } else {
            for (uint8_t i = 0; i < len; i++) _txBuffer[i] = buf[i];
        }

        _txState = 1; // TX_PENDING
        SET_FLAG(FLAG_IS_SENDING);

        return true;
    }

    template <uint8_t CAP>
    bool buff(SWP2PBuffer<CAP>& buf, bool bit)
    {
        return buf.pushBit(bit);
    }

    template <uint8_t CAP>
    bool buff(SWP2PBuffer<CAP>& buf, uint8_t byteVal, bool /*asByte*/)
    {
        return buf.pushByte(byteVal);
    }

    template <uint8_t CAP>
    void buffFree(SWP2PBuffer<CAP>& buf)
    {
        buf.clearAll();
    }

    template <uint8_t CAP>
    void buffFree(SWP2PBuffer<CAP>& buf, uint8_t idx)
    {
        buf.clearBuffer(idx);
    }

    template <uint8_t CAP>
    bool sendBurst(uint8_t destId, SWP2PBuffer<CAP>& buf)
    {
        static_assert(CAP <= SWP2P_MAX_BURST,
            "SWP2PBuffer CAP exceeds SWP2P_MAX_BURST - reduce CAP or increase SWP2P_MAX_BURST");

        return sendBurst(destId, buf.data, buf.len());
    }

    template <uint8_t CAP>
    bool sendBurst(uint8_t destId, SWP2PBuffer<CAP>& buf, uint8_t offset, uint8_t len)
    {
        static_assert(CAP <= SWP2P_MAX_BURST,
            "SWP2PBuffer CAP exceeds SWP2P_MAX_BURST - reduce CAP or increase SWP2P_MAX_BURST");

        if ((uint16_t)offset + len > buf.len()) return false;

        return sendBurst(destId, buf.data + offset, len);
    }

    bool available()
    {
        return _rxCount > 0;
    }

    // [발신자 ID 수정] data뿐 아니라 srcId도 반드시 같이 받아야 하도록 시그니처 자체를 강제함.
    // 큐가 비어있으면 false를 반환하고 data/srcId는 건드리지 않는다.
    bool read(uint8_t& data, uint8_t& srcId)
    {
        if (_rxCount == 0) return false;

        data = _rxFifo[_rxTail];
        srcId = _rxSrcFifo[_rxTail];
        // [xxxx xxxx] & [0000 1111] => [0000 xxxx] (Ring Buffer 인덱스 0~15 순환)
        _rxTail = (_rxTail + 1) & (SWP2P_FIFO_DEPTH - 1);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            _rxCount--;
        }

        return true;
    }

    // outBuf[i]를 받은 노드가 srcBuf[i]다 (인덱스가 1:1로 대응) — 패킷마다 발신자가 다를 수 있으므로
    // 데이터 배열만 받고 출처를 버리는 건 허용하지 않는다.
    uint8_t readBytes(uint8_t* outBuf, uint8_t* srcBuf, uint8_t maxLen)
    {
        uint8_t count = 0;
        while (count < maxLen && available()) {
            read(outBuf[count], srcBuf[count]);
            count++;
        }
        return count;
    }

    bool peek(uint8_t& data, uint8_t& srcId)
    {
        if (_rxCount == 0) return false;
        data = _rxFifo[_rxTail];
        srcId = _rxSrcFifo[_rxTail];
        return true;
    }

    void flush()
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            _rxHead = 0; _rxTail = 0; _rxCount = 0;
        }
    }

    bool isSending() const
    {
        return GET_FLAG(FLAG_IS_SENDING);
    }

    bool isBusy() const
    {
        return GET_FLAG(FLAG_IS_BUSY);
    }

    // [버그4 수정] 직전 sendBurst()/send()가 ACK 타임아웃으로 실패했는지 조회.
    // 읽는 순간 플래그를 자동으로 클리어한다 (다음 전송 결과와 섞이지 않도록).
    bool isAckFailed()
    {
        bool failed = GET_FLAG(FLAG_ACK_FAILED);
        CLR_FLAG(FLAG_ACK_FAILED);
        return failed;
    }

    // Compile-time Dispatching
    static inline void _driveDataChunk(uint8_t chunkVal) __attribute__((always_inline)) {
        // [DDR_reg] &= [BitMask_Clear] | [DataMask_Set] => 지정된 데이터 핀 방향 제어 (1:출력=0구동, 0:입력=하이임피던스)
        if constexpr (PRESET == PRESET_W1_D4) { DDRD = (DDRD & ~(1 << DDD4)) | ((~chunkVal & 0x01) << 4); }
        else if constexpr (PRESET == PRESET_W1_D5) { DDRD = (DDRD & ~(1 << DDD5)) | ((~chunkVal & 0x01) << 5); }
        else if constexpr (PRESET == PRESET_W1_D6) { DDRD = (DDRD & ~(1 << DDD6)) | ((~chunkVal & 0x01) << 6); }
        else if constexpr (PRESET == PRESET_W1_D7) { DDRD = (DDRD & ~(1 << DDD7)) | ((~chunkVal & 0x01) << 7); }
        else if constexpr (PRESET == PRESET_W1_D9) { DDRB = (DDRB & ~(1 << DDB1)) | ((~chunkVal & 0x01) << 1); }
        else if constexpr (PRESET == PRESET_W1_D10){ DDRB = (DDRB & ~(1 << DDB2)) | ((~chunkVal & 0x01) << 2); }
        else if constexpr (PRESET == PRESET_W1_A0) { DDRC = (DDRC & ~(1 << DDC0)) | (~chunkVal & 0x01); }
        else if constexpr (PRESET == PRESET_W2_D4_D5) { DDRD = (DDRD & ~0x30) | ((~chunkVal & 0x03) << 4); }
        else if constexpr (PRESET == PRESET_W2_D6_D7) { DDRD = (DDRD & ~0xC0) | ((~chunkVal & 0x03) << 6); }
        else if constexpr (PRESET == PRESET_W2_D9_D10){ DDRB = (DDRB & ~0x06) | ((~chunkVal & 0x03) << 1); }
        else if constexpr (PRESET == PRESET_W2_A0_A1) { DDRC = (DDRC & ~0x03) | (~chunkVal & 0x03); }
        else if constexpr (PRESET == PRESET_W4_D4_D7) { DDRD = (DDRD & ~0xF0) | ((~chunkVal & 0x0F) << 4); }
        else if constexpr (PRESET == PRESET_W4_A0_A3) { DDRC = (DDRC & ~0x0F) | (~chunkVal & 0x0F); }
        else if constexpr (PRESET == PRESET_W4_D9_D12) { DDRB = (DDRB & ~0x1E) | ((~chunkVal & 0x0F) << 1); }
        else if constexpr (PRESET == PRESET_W8_A0_D7) {
            DDRC = (DDRC & ~0x0F) | (~chunkVal & 0x0F);
            DDRD = (DDRD & ~0xF0) | ((~chunkVal & 0xF0));
        }
    }

    static inline uint8_t _readDataChunk() __attribute__((always_inline)) {
        uint8_t val = 0;
        // [PIN_reg] >> [Shift] & [BitMask] => 데이터 핀 상태만 0~N 비트 크기로 정렬하여 수신
        if constexpr (PRESET == PRESET_W1_D4)   val = (PIND >> PIND4) & 0x01;
        else if constexpr (PRESET == PRESET_W1_D5)  val = (PIND >> PIND5) & 0x01;
        else if constexpr (PRESET == PRESET_W1_D6)  val = (PIND >> PIND6) & 0x01;
        else if constexpr (PRESET == PRESET_W1_D7)  val = (PIND >> PIND7) & 0x01;
        else if constexpr (PRESET == PRESET_W1_D9)  val = (PINB >> PINB1) & 0x01;
        else if constexpr (PRESET == PRESET_W1_D10) val = (PINB >> PINB2) & 0x01;
        else if constexpr (PRESET == PRESET_W1_A0)  val = (PINC >> PINC0) & 0x01;
        else if constexpr (PRESET == PRESET_W2_D4_D5) val = (PIND & 0x30) >> 4;
        else if constexpr (PRESET == PRESET_W2_D6_D7) val = (PIND & 0xC0) >> 6;
        else if constexpr (PRESET == PRESET_W2_D9_D10)val = (PINB & 0x06) >> 1;
        else if constexpr (PRESET == PRESET_W2_A0_A1) val = (PINC & 0x03);
        else if constexpr (PRESET == PRESET_W4_D4_D7) val = (PIND & 0xF0) >> 4;
        else if constexpr (PRESET == PRESET_W4_A0_A3) val = (PINC & 0x0F);
        else if constexpr (PRESET == PRESET_W4_D9_D12) val = (PINB & 0x1E) >> 1;
        else if constexpr (PRESET == PRESET_W8_A0_D7) val = (PINC & 0x0F) | (PIND & 0xF0);
        return val;
    }

    static inline void _dataRelease() __attribute__((always_inline)) {
        // [DDR_reg] &= ~(BitMask) => 데이터 핀을 모두 입력(High-Z)으로 해제
        if constexpr (PRESET == PRESET_W1_D4) DDRD &= ~(1 << DDD4);
        else if constexpr (PRESET == PRESET_W1_D5) DDRD &= ~(1 << DDD5);
        else if constexpr (PRESET == PRESET_W1_D6) DDRD &= ~(1 << DDD6);
        else if constexpr (PRESET == PRESET_W1_D7) DDRD &= ~(1 << DDD7);
        else if constexpr (PRESET == PRESET_W1_D9) DDRB &= ~(1 << DDB1);
        else if constexpr (PRESET == PRESET_W1_D10) DDRB &= ~(1 << DDB2);
        else if constexpr (PRESET == PRESET_W1_A0) DDRC &= ~(1 << DDC0);
        else if constexpr (PRESET == PRESET_W2_D4_D5) DDRD &= ~0x30;
        else if constexpr (PRESET == PRESET_W2_D6_D7) DDRD &= ~0xC0;
        else if constexpr (PRESET == PRESET_W2_D9_D10) DDRB &= ~0x06;
        else if constexpr (PRESET == PRESET_W2_A0_A1) DDRC &= ~0x03;
        else if constexpr (PRESET == PRESET_W4_D4_D7) DDRD &= ~0xF0;
        else if constexpr (PRESET == PRESET_W4_A0_A3) DDRC &= ~0x0F;
        else if constexpr (PRESET == PRESET_W4_D9_D12) DDRB &= ~0x1E;
        else if constexpr (PRESET == PRESET_W8_A0_D7) { DDRC &= ~0x0F; DDRD &= ~0xF0; }
    }

    static constexpr uint8_t DATA_WIDTH =
        (PRESET >= PRESET_W8_A0_D7) ? 8 :
        (PRESET >= PRESET_W4_D4_D7) ? 4 :
        (PRESET >= PRESET_W2_D4_D5) ? 2 : 1;

    static constexpr uint8_t ARB_CYCLES = (8 + DATA_WIDTH - 1) / DATA_WIDTH;

    // [버그4 수정] ACK 대기 타임아웃 (falling edge 기준 카운트). 수신측은 RX_DATA 마지막 청크를
    // 읽는 바로 그 순간(같은 클럭 사이클)에 곧장 PB0을 LOW로 구동하므로 이론상 거의 즉시 보여야
    // 하지만, 지터/노이즈 캔슬러 지연 여유를 감안해 몇 edge 정도는 기다려준다.
    static constexpr uint8_t ACK_WAIT_EDGES = 4;

    // [2^N - 1 원리] (1 << DATA_WIDTH) - 1: 하위 N비트(DATA_WIDTH 크기)만 1로 채워진 거름망 마스크 생성
    // N=1: (1<<1)-1 = 0b00000001 (0x01)
    // N=2: (1<<2)-1 = 0b00000011 (0x03)
    // N=4: (1<<4)-1 = 0b00001111 (0x0F)
    // N=8: (1<<8)-1 = 0b11111111 (0xFF)
    static constexpr uint8_t DATA_MASK = (1 << DATA_WIDTH) - 1;

    static constexpr uint8_t ARB_SHIFT = 16 - DATA_WIDTH;
    static constexpr uint8_t TX_SHIFT = 8 - DATA_WIDTH;

    // ============================================================
    // CLK 엣지 핸들러 (INT0)
    // ============================================================
    static void _onClkEdge() {
        // [PIND] & [0000 0100] => [0000 0x00] (CLK=PD2 하이 상태 확인)
        bool clkHigh = (PIND & (1 << PIND2)) != 0;

        if (clkHigh) {
            if (_txState == 1) { // TX_PENDING
                PORTD &= ~(1 << PORTD3); // [xxxx xxxx] &= [1111 0111] => [xxxx 0xxx] (D3 LOW)
                DDRD |= (1 << DDD3);     // [xxxx xxxx] |= [0000 1000] => [xxxx 1xxx] (D3 BUSY 출력 켜서 라인 점유)

                uint16_t arbReg = ((uint16_t)_txDestId << 8) | _nodeId;
                // [시프트 & DATA_MASK] 상위 비트를 당겨온 후, DATA_MASK(2^N-1)와 & 연산하여 보낼 청크만 추출
                uint8_t myChunk = (arbReg >> ARB_SHIFT) & DATA_MASK;
                arbReg <<= DATA_WIDTH;                                // 다음 청크 전송 준비
                _driveDataChunk(myChunk);
                _txArbReg = arbReg;
                _arbMyChunk = myChunk;
                _arbChunksSent = 1; // [버그1 수정] 이번이 첫 청크
                _arbChunkCount = (2 * ARB_CYCLES) - 1;
                _txState = 2; // TX_ARB
                return;
            }

            if (_txState == 2) { // TX_ARB
                uint8_t chunkCnt = _arbChunkCount;
                if (chunkCnt == 0) {
                    if (_txLen > 1) {
                        _txDataReg = _txLen - 2;
                        _txDataChunkCount = ARB_CYCLES;
                        // [시프트 & DATA_MASK] MSB 비트들을 하위로 내린 후 2^N-1 마스크로 필요 크기만 추출
                        uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                        _txDataReg <<= DATA_WIDTH;
                        _driveDataChunk(chunk);
                        _txDataChunkCount--;
                        _txIdx = 0;
                        _txState = 3; // TX_LEN
                    } else {
                        _txDataReg = _txData;
                        _txDataChunkCount = ARB_CYCLES;
                        uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK; // 2^N-1 마스킹 추출
                        _txDataReg <<= DATA_WIDTH;
                        _driveDataChunk(chunk);
                        _txDataChunkCount--;
                        _txState = 4; // TX_DATA
                    }
                } else {
                    uint16_t arbReg = _txArbReg;
                    uint8_t myChunk = (arbReg >> ARB_SHIFT) & DATA_MASK; // 2^N-1 마스킹 추출
                    arbReg <<= DATA_WIDTH;
                    _driveDataChunk(myChunk);
                    _txArbReg = arbReg;
                    _arbMyChunk = myChunk;
                    _arbChunksSent++; // [버그1 수정]
                    _arbChunkCount = chunkCnt - 1;
                }
            } else if (_txState == 3) { // TX_LEN
                if (_txDataChunkCount == 0) {
                    _txDataReg = _txBuffer[0];
                    _txDataChunkCount = ARB_CYCLES;
                    uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK; // 2^N-1 마스킹 추출
                    _txDataReg <<= DATA_WIDTH;
                    _driveDataChunk(chunk);
                    _txDataChunkCount--;
                    _txState = 4; // TX_DATA
                } else {
                    uint8_t dataReg = _txDataReg;
                    uint8_t chunk = (dataReg >> TX_SHIFT) & DATA_MASK; // 2^N-1 마스킹 추출
                    dataReg <<= DATA_WIDTH;
                    _driveDataChunk(chunk);
                    _txDataReg = dataReg;
                    _txDataChunkCount--;
                }
            } else if (_txState == 4) { // TX_DATA
                if (_txDataChunkCount == 0) {
                    _txIdx++;
                    if (_txLen > 1 && _txIdx < _txLen) {
                        _txDataReg = _txBuffer[_txIdx];
                        _txDataChunkCount = ARB_CYCLES;
                        uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK; // 2^N-1 마스킹 추출
                        _txDataReg <<= DATA_WIDTH;
                        _driveDataChunk(chunk);
                        _txDataChunkCount--;
                    } else {
                        // [버그4 수정] 원래는 DATA 다 보내자마자 바로 BUSY를 풀고 "성공"으로 간주했음
                        // (상대가 실제로 받았는지 전혀 확인 안 함). 데이터 라인은 지금 놓아주되,
                        // BUSY(D3)는 계속 잡고 ACK 대기 상태(7)로 넘어가서 PB0(ACK)이 LOW로
                        // 내려오는지 확인한다.
                        _dataRelease();
                        _ackWaitCount = ACK_WAIT_EDGES;
                        _txState = 7; // TX_WAIT_ACK
                    }
                } else {
                    uint8_t dataReg = _txDataReg;
                    uint8_t chunk = (dataReg >> TX_SHIFT) & DATA_MASK; // 2^N-1 마스킹 추출
                    dataReg <<= DATA_WIDTH;
                    _driveDataChunk(chunk);
                    _txDataReg = dataReg;
                    _txDataChunkCount--;
                }
            }
            // (구 TX_RELEASE(state 5)는 [버그4 수정]으로 TX_WAIT_ACK(state 7)이 릴리즈까지
            //  직접 처리하게 되면서 더 이상 쓰이지 않음 - falling-edge 쪽 state7 핸들러 참고)
        } else {
            if (_txState == 7) { // [버그4 수정] TX_WAIT_ACK negedge: ACK(PB0=D8) 확인
                bool acked = (PINB & (1 << PINB0)) == 0; // 수신측이 PB0을 LOW로 구동했는지
                if (acked) {
                    CLR_FLAG(FLAG_ACK_FAILED);
                    DDRD &= ~(1 << DDD3); // BUSY 해제
                    CLR_FLAG(FLAG_IS_SENDING);
                    _txState = 0;
                } else if (_ackWaitCount == 0) {
                    // 타임아웃: ACK 못 받았지만 버스는 풀어줘야 다른 노드가 계속 쓸 수 있음
                    SET_FLAG(FLAG_ACK_FAILED);
                    DDRD &= ~(1 << DDD3);
                    CLR_FLAG(FLAG_IS_SENDING);
                    _txState = 0;
                } else {
                    _ackWaitCount--;
                }
                return;
            }
            if (_txState == 2) { // TX_ARB negedge: 중재 충돌 검증
                uint8_t busVal = _readDataChunk();
                // [내가 보낸 비트] & ~[버스의 실제 비트] => 내가 0(구동)인데 버스가 1이면 충돌(패배)
                if ((_arbMyChunk & ~busVal) != 0) {
                    _dataRelease();
                    DDRD &= ~(1 << DDD3); // [xxxx xxxx] &= [1111 0111] => [xxxx 0xxx] (BUSY 해제 - 내 쪽만. 승자가 계속 잡고있으므로 버스 자체는 그대로 busy)
                    CLR_FLAG(FLAG_IS_SENDING);

                    // [버그1 수정] 중재 패배 = 지금 이 순간 "더 높은 우선순위 노드가 주소 필드를
                    // 보내는 중"이라는 뜻이고, 그 패킷의 목적지가 나 자신일 수도 있다.
                    // 그런데 그냥 _rxState=0으로 방치하면 이 패킷의 남은 부분(주소 뒷부분/SRC/LEN/DATA)을
                    // 통째로 놓쳐버린다. 지금까지 비교를 통과한(=충돌 안 난) 청크들은 전부
                    // "내가 보낸 값 == 실제 버스 값"이었다는 뜻이므로, 내가 보내려 했던 원본 주소값을
                    // 그대로 재사용해서 복원할 수 있다. 단, 지금 막 충돌이 감지된 이 청크 하나만은
                    // 방금 읽은 busVal이 진짜 값이므로 그걸로 교체한다.
                    {
                        uint16_t myOriginal = ((uint16_t)_txDestId << 8) | _nodeId; // 내가 보내려던 16비트 주소(destId<<8 | srcId)
                        uint8_t doneBits = _arbChunksSent * DATA_WIDTH; // 지금까지(이 청크 포함) 확정된 비트 수
                        // myOriginal의 상위 doneBits비트를 오른쪽 정렬로 뽑아낸 뒤, 마지막 청크(현재 충돌한 것)만
                        // busVal로 교정 -> "지금까지 실제로 오간 값" 그대로 복원됨 (accumulator가 원래
                        // 왼쪽시프트+OR로 채워나가는 것과 동일한 오른쪽 정렬 방식)
                        uint16_t known = myOriginal >> (16 - doneBits);
                        known = (uint16_t)(known & ~(uint16_t)DATA_MASK) | busVal;

                        if (doneBits <= 8) {
                            // 아직 destId 필드 진행 중(또는 막 끝난 시점)
                            _rxAddrByte = (uint8_t)known;
                            uint8_t remain = (8 - doneBits) / DATA_WIDTH;
                            if (remain == 0) {
                                // destId 필드가 이 청크로 끝남 -> RX_ADDR 완료 처리를 그대로 이어서 수행
                                uint8_t addr7 = _rxAddrByte & SWP2P_NODE_MASK;
                                bool isBurst = (_rxAddrByte & SWP2P_BURST_BIT) != 0;
                                if (isBurst) SET_FLAG(FLAG_RX_IS_BURST); else CLR_FLAG(FLAG_RX_IS_BURST);
                                if (addr7 == _nodeId || addr7 == SWP2P_BROADCAST) SET_FLAG(FLAG_IS_MY_PACKET);
                                else CLR_FLAG(FLAG_IS_MY_PACKET);
                                _rxSrcByte = 0;
                                _rxChunkCount = ARB_CYCLES;
                                _rxState = 2; // RX_SRC
                            } else {
                                _rxChunkCount = remain;
                                _rxState = 1; // RX_ADDR (이어서 계속)
                            }
                        } else {
                            // 이미 srcId 필드로 넘어간 시점 -> destId는 이미 전부 확정됨(=내가 보내려던 destId 그대로,
                            // 여기까지 충돌이 없었다는 것 자체가 그 증거)
                            uint8_t addr7 = _txDestId & SWP2P_NODE_MASK;
                            bool isBurst = (_txDestId & SWP2P_BURST_BIT) != 0;
                            if (isBurst) SET_FLAG(FLAG_RX_IS_BURST); else CLR_FLAG(FLAG_RX_IS_BURST);
                            if (addr7 == _nodeId || addr7 == SWP2P_BROADCAST) SET_FLAG(FLAG_IS_MY_PACKET);
                            else CLR_FLAG(FLAG_IS_MY_PACKET);

                            uint8_t srcDoneBits = doneBits - 8;
                            _rxSrcByte = (uint8_t)(known & (((uint16_t)1 << srcDoneBits) - 1));
                            uint8_t remain = (8 - srcDoneBits) / DATA_WIDTH;
                            if (remain == 0) {
                                // srcId까지 이 청크로 끝남 -> RX_SRC 완료 처리를 그대로 이어서 수행
                                if (GET_FLAG(FLAG_RX_IS_BURST)) {
                                    _rxDataByte = 0; _rxChunkCount = ARB_CYCLES; _rxState = 3; // RX_LEN
                                } else {
                                    _rxDataByte = 0; _rxChunkCount = ARB_CYCLES; _rxLen = 1; _rxByteIdx = 0; _rxState = 4; // RX_DATA
                                }
                            } else {
                                _rxChunkCount = remain;
                                _rxState = 2; // RX_SRC (이어서 계속)
                            }
                        }
                    }

                    _txState = 6; // TX_LOST
                }
                return;
            }
            // [버그1 수정] TX_LOST(6) 상태에서도 RX 처리는 계속 흘러가야 한다 (원래는 _txState!=0이면
            // 무조건 return이라 TX_LOST 동안 RX 상태머신이 완전히 멈춰서 승자의 패킷을 못 받았음).
            if (_txState != 0 && _txState != 6) return;

            uint8_t rxState = _rxState;

            if (rxState == 4) { // RX_DATA
                uint8_t v = _readDataChunk();
                // [기존 수신 데이터 << DATA_WIDTH] | [새 수신 청크 v] => 바이트 조립
                uint8_t dataByte = (_rxDataByte << DATA_WIDTH) | v;
                uint8_t chunkCnt = _rxChunkCount - 1;
                if (chunkCnt == 0) {
                    if (GET_FLAG(FLAG_IS_MY_PACKET)) {
                        _fifoPush(dataByte, _rxSrcByte); // [발신자 ID 수정] RX_SRC 단계에서 이미 확정된 발신자 ID를 데이터와 함께 큐에 넣음
                        SET_FLAG(FLAG_RX_CAPTURED);
                    }
                    _rxByteIdx++;
                    if (_rxByteIdx < _rxLen) {
                        _rxDataByte = 0;
                        _rxChunkCount = ARB_CYCLES;
                    } else {
                        _rxState = 5; // RX_ACK
                    }
                } else {
                    _rxDataByte = dataByte;
                    _rxChunkCount = chunkCnt;
                }
            } else if (rxState == 1) { // RX_ADDR
                uint8_t v = _readDataChunk();
                _rxAddrByte = (_rxAddrByte << DATA_WIDTH) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    uint8_t addr7 = _rxAddrByte & SWP2P_NODE_MASK; // [xxxx xxxx] & [0111 1111] => [0xxx xxxx] (7비트 주소)
                    bool isBurst = (_rxAddrByte & SWP2P_BURST_BIT) != 0; // [xxxx xxxx] & [1000 0000] => MSB 1인지 체크
                    if (isBurst) SET_FLAG(FLAG_RX_IS_BURST); else CLR_FLAG(FLAG_RX_IS_BURST);
                    if (addr7 == _nodeId || addr7 == SWP2P_BROADCAST) {
                        SET_FLAG(FLAG_IS_MY_PACKET);
                    } else {
                        CLR_FLAG(FLAG_IS_MY_PACKET);
                    }
                    _rxSrcByte = 0;
                    _rxChunkCount = ARB_CYCLES;
                    _rxState = 2; // RX_SRC
                }
            } else if (rxState == 2) { // RX_SRC
                uint8_t v = _readDataChunk();
                _rxSrcByte = (_rxSrcByte << DATA_WIDTH) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    if (GET_FLAG(FLAG_RX_IS_BURST)) {
                        _rxDataByte = 0;
                        _rxChunkCount = ARB_CYCLES;
                        _rxState = 3; // RX_LEN
                    } else {
                        _rxDataByte = 0;
                        _rxChunkCount = ARB_CYCLES;
                        _rxLen = 1;
                        _rxByteIdx = 0;
                        _rxState = 4; // RX_DATA
                    }
                }
            } else if (rxState == 3) { // RX_LEN
                uint8_t v = _readDataChunk();
                _rxDataByte = (_rxDataByte << DATA_WIDTH) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    _rxLen = _rxDataByte + 2;
                    _rxByteIdx = 0;
                    _rxDataByte = 0;
                    _rxChunkCount = ARB_CYCLES;
                    _rxState = 4; // RX_DATA
                }
            } else if (rxState == 5) { // RX_ACK
                // [버그4 전제 수정] 원래는 addressing 여부와 무관하게 무조건 ACK를 쏴서,
                // 모든 리슨 노드가 매 패킷마다 ACK를 구동했음(= ACK가 "내가 받았다"는 증거로 쓸모없었음).
                // 실제 목적지였던 노드만 ACK를 구동하도록 게이트.
                if (GET_FLAG(FLAG_IS_MY_PACKET)) {
                    PORTB &= ~(1 << PORTB0); // [xxxx xxxx] &= [1111 1110] => [xxxx xxx0] (PB0 LOW)
                    DDRB |= (1 << DDB0);     // [xxxx xxxx] |= [0000 0001] => [xxxx xxx1] (PB0 ACK 응답 구동)
                }
                _rxState = 0;
            }
        }
    }

    static void _onBusyEdge() {
        // [PIND] & [0000 1000] => [0000 x000] (BUSY=PD3 상태 확인)
        bool idle = (PIND & (1 << PIND3)) != 0;
        if (idle) {
            CLR_FLAG(FLAG_IS_BUSY);
            DDRB &= ~(1 << DDB0); // [xxxx xxxx] &= [1111 1110] => [xxxx xxx0] (PB0 입력/해제)
            CLR_FLAG(FLAG_RX_CAPTURED);
            CLR_FLAG(FLAG_RX_IS_BURST);
            _rxState = 0;
            _rxAddrByte = 0;
            _rxSrcByte = 0;
            _rxLen = 0;
            _rxByteIdx = 0;

            if (_txState == 6) { // TX_LOST -> 재시도
                _txState = 1;
                SET_FLAG(FLAG_IS_SENDING);
            }
        } else {
            SET_FLAG(FLAG_IS_BUSY);
            if (_txState == 0) {
                _rxState = 1;
                _rxAddrByte = 0;
                _rxChunkCount = ARB_CYCLES;
            }
        }
    }

    static void _onAckCapture() {
        // [TCCR1B] ^= [0100 0000] => ICES1( bit6 ) 반전 (Rising Edge <-> Falling Edge 캡처 극성 토글)
        TCCR1B ^= (1 << ICES1);
    }
};

#define SWP2P_BIND_ISRS(PRESET_TYPE) \
    ISR(INT0_vect)        { SWP2P<PRESET_TYPE>::_onClkEdge(); } \
    ISR(INT1_vect)        { SWP2P<PRESET_TYPE>::_onBusyEdge(); } \
    ISR(TIMER1_CAPT_vect) { SWP2P<PRESET_TYPE>::_onAckCapture(); }

#endif
