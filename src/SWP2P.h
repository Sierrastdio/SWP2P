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
#include "SWP2PPreset.h" // DataPreset enum, 프리셋별 핀 조작(_driveFrameChunk 등), WIDTH 관련 상수는 전부 여기로 이동됨

#define SWP2P_FIFO_DEPTH 16
#define SWP2P_MAX_BURST   16  // TX buffer capacity; LEN width is derived at compile time (1..255).
                              // RAM 절약 위해 우선 16으로 제한 (필요시 늘리면 됨).
#define SWP2P_MAX_SLOTS   63  // 슬롯 중재 최대 슬롯 수 (최대 노드 ID와 동일: 0~62)

// 라운드로빈 공정성 옵션 (0: 고정 ID 슬롯 우선순위, 1: 직전 송신 노드 다음 노드부터 슬롯 시작)
#ifndef SWP2P_ROUND_ROBIN
#define SWP2P_ROUND_ROBIN 0
#endif

// ---- 주소 인코딩 ----
// Wire header is DEST(6), SRC(6), BURST(1), MSB first; IDs 0..62, 63 broadcast.
// SWP2P_BURST_BIT is an internal marker in _txDestId, not a wire-byte bit.
#define SWP2P_NODE_MASK   0x3F  // 0011 1111
#define SWP2P_BURST_BIT   0x80
#define SWP2P_BROADCAST   0x3F  // 0011 1111

#define FLAG_IS_SENDING   0
#define FLAG_IS_BUSY      1
#define FLAG_IS_MY_PACKET 3
#define FLAG_ACK_FAILED   5   // 직전 송신이 ACK를 못 받고 타임아웃됐는지 (isAckFailed()로 조회)

// GPIOR0 플래그 비트 연산 추적 주석 추가
#define SET_FLAG(b) (GPIOR0 |= (1 << (b))) // 플래그 비트 b 마스크: 0000 0001 << b
#define CLR_FLAG(b) (GPIOR0 &= ~(1 << (b))) // 플래그 비트 b 마스크: 0000 0001 << b
#define GET_FLAG(b) (GPIOR0 & (1 << (b))) // 플래그 비트 b 마스크: 0000 0001 << b

// DataPreset enum 및 Arduino 핀아웃 관련 내용은 SWP2PPreset.h로 이동됨

// ---- Tx/Rx 상태 번호 ----
// Tx: 0 IDLE, 1 PENDING, 2 HEADER, 3 LEN, 4 DATA, 7 WAIT_ACK
// Rx: 0 IDLE/filtered, 1 packed header, 3 LEN, 4 addressed DATA, 5 ACK, 6 clock-master discard

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

    static uint8_t _txDestId;      // 6-bit destination plus temporary burst flag
    static uint8_t _txData;        // len==1일 때만 사용 (기존 빠른 경로 그대로 유지)
    static uint8_t _txBuffer[SWP2P_MAX_BURST]; // len>1일 때 사용
    static uint8_t _txLen;
    static uint8_t _txIdx;
    // [속도 개선] 아래 필드들은 _onClkEdge()/_onBusyEdge()(ISR 브릿지) 안에서만 읽고 쓴다.
    // send()/read()/isSending() 등 공개 API나 메인 루프가 동시에 건드리는 일이 없으므로
    // volatile을 뗐다 - volatile이 붙으면 같은 함수 안에서도 매번 SRAM으로 다시 읽고/쓰게
    // 강제되어(레지스터 캐싱 금지) 에지마다 불필요한 load/store가 늘어난다.
    static uint16_t _txArbReg;     // 헤더 push-pull 송신 시 남은 비트를 보관하는 시프트 레지스터
    static uint8_t _txDataReg;   // data byte or left-aligned LEN bits
    static uint8_t _arbChunkCount;
    static uint8_t _txDataChunkCount;
    static uint8_t _ackWaitCount;  // [버그4 수정] ACK 대기 중 남은 edge 수 (0이면 타임아웃)

    static uint8_t _slotCounter;   // 슬롯 중재: 버스 idle 중 CLK 상승 에지마다 증가, idle 복귀 시 리셋
#if SWP2P_ROUND_ROBIN
    static uint8_t _slotBase;      // 라운드로빈 슬롯 시작 오프셋
    static uint8_t _lastSenderId;  // 직전 프레임 송신자 ID
#endif

    static uint8_t _rxAddrByte; // high byte of the packed 13-bit arbitration header
    static uint8_t _rxSrcByte;
    static uint8_t _rxDataByte;   // data byte or packed LEN accumulator
    static uint8_t _rxChunkCount;
    static uint8_t _rxLen;        // 이번 프레임에서 받아야 할 총 바이트 수
    static uint8_t _rxByteIdx;    // 지금까지 받은 바이트 수

    static void _fifoPush(uint8_t val, uint8_t srcId); // [발신자 ID 수정]
    static void setupTimer1(unsigned long freq);
    static void stopTimer1();

    // ---- Dynamic Clock Scaling (동적 클럭 가변) ----
    // CLK을 실제로 만드는 건 begin(true, ...)로 초기화된 딱 한 노드의 Timer1(CTC + COM1A0
    // 하드웨어 자동 토글)뿐이다. 나머지 노드는 CLK을 그냥 받아서 따라갈 뿐이라 여기서 아무것도
    // 할 수 없다 - 그래서 _isClkMaster가 false면 _clkGoFast/_clkGoSlow는 완전한 no-op이어야 한다.
    static volatile bool _isClkMaster;
    static uint16_t _ocrArb;   // HEADER(슬롯 중재)/ACK 구간용 저속 OCR1A (안전 최우선)
    static uint16_t _ocrData; // DATA(페이로드) 구간용 고속 OCR1A (실측으로 튜닝해서 넣을 값)

    // freq(Hz) -> OCR1A 값 변환. setupTimer1()의 공식과 동일해야 한다(분주비 1 고정 전제).
    // [버그 수정] OCR1A는 16비트라서 분주비 1 기준 표현 가능한 범위가 있다:
    //   - 최저 주파수 ≈ F_CPU/(2*65536) (16MHz 기준 약 122Hz) 미만을 넣으면
    //     (F_CPU/(2*freq))-1 이 65535를 넘어서 uint16_t로 캐스팅될 때 조용히 오버플로우/랩어라운드
    //     되어 완전히 엉뚱한(훨씬 높은) 주파수가 나간다 - 에러도 안 나고 그냥 틀린 값으로 동작해버림.
    //   - 반대로 너무 높은 주파수를 넣으면 raw가 0이 되어 -1 시점에 uint16_t 언더플로우(65535)가 난다.
    // 그래서 여기서 범위를 클램프해 "조용히 틀린 값"이 되는 상황 자체를 차단한다.
    static inline uint16_t _freqToOcr(unsigned long freq) {
        if (freq == 0) freq = 1; // 0 나눗셈 방지
        unsigned long raw = F_CPU / (2UL * freq);
        if (raw < 1UL) raw = 1UL;         // 표현 가능한 최고 주파수 한계 (OCR1A=0)
        if (raw > 65536UL) raw = 65536UL; // 표현 가능한 최저 주파수 한계 (OCR1A=65535, 16MHz 기준 약 122Hz)
        return (uint16_t)(raw - 1UL);
    }

    // HEADER(슬롯 중재) 완료 후 LEN/DATA로 넘어가는 그 순간(=에지 ISR 진입 직후, TCNT1이 막 0으로
    // 리셋된 시점) 호출한다. TCNT1을 여기서 같이 0으로 리셋해주면
    // ISR 진입~여기 도달 시간과 무관하게 항상 "지금부터 정확히 새 주기"가 보장된다.
    static inline void _clkGoFast() __attribute__((always_inline)) {
        if (_isClkMaster) { OCR1A = _ocrData; TCNT1 = 0; }
    }
    // DATA가 끝나고 ACK/유휴로 돌아가는 순간, 그리고 버스 idle 리셋 시점에 호출해서 항상
    // HEADER에 안전한 저속으로 복귀시킨다. (위와 동일한 이유로 TCNT1도 같이 리셋)
    static inline void _clkGoSlow() __attribute__((always_inline)) {
        if (_isClkMaster) { OCR1A = _ocrArb; TCNT1 = 0; }
    }
};

template <DataPreset PRESET>
class SWP2P : public SWP2PBase {
public:
    static constexpr uint8_t BROADCAST = SWP2P_BROADCAST;

    SWP2P(uint8_t nodeId)
    {
        // Keep the broadcast value reserved for destination addressing.
        _nodeId = nodeId & SWP2P_NODE_MASK;
        if (_nodeId == SWP2P_BROADCAST) --_nodeId;
    }

    // dataFreq를 생략하면(0) clkFreq와 동일하게 취급되어 기존과 완전히 동일하게 동작한다
    // (=동적 클럭 기능을 켜지 않은 기존 사용자 코드는 아무 영향 없음).
    // dataFreq를 지정하면: HEADER/ACK 구간은 clkFreq(저속, 안전 우선), DATA(페이로드) 구간만
    // dataFreq(고속)로 잠깐 가속했다가 ACK 대기에 들어가는 순간 다시 clkFreq로 돌아온다.
    // dataFreq 값은 "이론상 최댓값"이 아니라 실측으로 정해야 한다 - 이 버스에 붙은 노드 중
    // 가장 느린 노드(WIDTH가 넓을수록, ISR이 무거울수록 느림)가 감내 못하면 에지를 놓쳐
    // 프레임이 깨진다. CRC 없이 속도를 올리면 깨진 데이터도 조용히 통과할 수 있으니 주의.
    void begin(bool clkIsOutput, unsigned long clkFreq = 100000UL, unsigned long dataFreq = 0)
    {
        GPIOR0 = 0;
        _txState = 0; // GPIOR1
        _rxState = 0; // GPIOR2

        _isClkMaster = clkIsOutput;
        _ocrArb  = _freqToOcr(clkFreq);
        _ocrData = _freqToOcr(dataFreq != 0 ? dataFreq : clkFreq);

        // PD2 입력 설정: DDRD &= ~(1 << PD2)
        DDRD &= ~(1 << PD2);   // PD2, bit 2: 0000 0100
        // PD2 풀업 해제: PORTD &= ~(1 << PD2)
        PORTD &= ~(1 << PD2); // PD2, bit 2: 0000 0100

        PORTD &= ~(1 << PD3); // 마스크 PD3, bit 3: 0000 1000; D3 출력 0
        DDRD &= ~(1 << PD3);   // 마스크 PD3, bit 3: 0000 1000; D3 입력 설정
        PORTB &= ~(1 << PB0); // 마스크 PB0, bit 0: 0000 0001; PB0 출력 0
        DDRB &= ~(1 << PB0);   // 마스크 PB0, bit 0: 0000 0001; PB0 입력 설정

        _frameDataRelease();

        _slotCounter = 0;
#if SWP2P_ROUND_ROBIN
        _slotBase = 0;
        _lastSenderId = 0;
#endif

        if (clkIsOutput) {
            // 안전 가드: D9(PB1)를 데이터 핀으로 사용하는 프리셋은 Timer1 OC1A(PB1) 출력과
            // 핀이 겹치므로 CLK 마스터로 동작할 수 없음 (슬레이브 begin(false)로만 사용 가능)
            // PRESET은 템플릿 상수라 최적화 시 상수 조건으로 접힌다. if constexpr는
            // Arduino AVR 기본 gnu++11 설정에서 컴파일되지 않으므로 일반 if를 사용한다.
            if (PRESET == PRESET_W1_D9 || PRESET == PRESET_W2_D9_D10 || PRESET == PRESET_W4_D9_D12) {
                return;
            }
            DDRB |= (1 << PB1); // 마스크 PB1, bit 1: 0000 0010; PB1 출력 설정
            setupTimer1(clkFreq);
        } else {
            TCCR1A = 0; // 초기화 마스크: 0000 0000
            TCCR1B = (1 << CS10); // CS10, bit 0: 0000 0001 (Timer1 분주비 1 설정)
        }

        // TCCR1B: 노이즈 캔슬러(ICNC1) 및 엣지 세팅
        TCCR1B |= (1 << ICNC1); // ICNC1, bit 7: 1000 0000 (노이즈 캔슬러 ON)
        TCCR1B |= (1 << ICES1); // ICES1, bit 6: 0100 0000 (Rising Edge 캡처)
        TIMSK1 |= (1 << ICIE1); // ICIE1, bit 5: 0010 0000 (Capture 인터럽트 ON)

        // EICRA: INT0 설정 (ISC01, ISC00 비트)
        EICRA &= ~((1 << ISC01) | (1 << ISC00)); // ISC01: 0000 0010, ISC00: 0000 0001; 함께 0000 0011 (INT0 초기화)
        EICRA |= (1 << ISC00); // ISC00, bit 0: 0000 0001 (INT0 Logical Change)
        EIMSK |= (1 << INT0); // INT0, bit 0: 0000 0001 (INT0 Enable)

        // EICRA: INT1 설정 (ISC11, ISC10 비트)
        // 주의: Low Level(00)로 두면 D3(BUSY)가 LOW인 "동안"만 인터럽트가 걸리고, HIGH로
        // 복귀하는 상승 엣지(=버스 idle 전환)를 감지할 방법이 없다. _onBusyEdge()의
        // idle 분기(FLAG_IS_BUSY를 푸는 유일한 지점)가 바로 그 상승 엣지에서 실행돼야
        // 하므로, Any Logical Change(01)로 바꿔서 양쪽 엣지를 모두 잡아야 한다.
        // (이게 없으면 최초 1회 통신 후 FLAG_IS_BUSY가 영구히 안 풀림 -> 이후 전송 전부 조용히 무시됨)
        EICRA &= ~((1 << ISC11) | (1 << ISC10)); // ISC11: 0000 1000, ISC10: 0000 0100; 함께 0000 1100 (INT1 초기화)
        EICRA |= (1 << ISC10); // ISC10, bit 2: 0000 0100 (INT1 Any Logical Change)
        EIMSK |= (1 << INT1); // INT1, bit 1: 0000 0010 (INT1 Enable)

        sei();
    }

    bool send(uint8_t destId, uint8_t data)
    {
        return sendBurst(destId, &data, 1);
    }

    bool sendBurst(uint8_t destId, const uint8_t* buf, uint8_t len)
    {
        if (buf == nullptr || len == 0 || len > SWP2P_MAX_BURST) return false;

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

    // 프리셋별 핀 조작은 SWP2PPreset.h의 free 함수로 이동됨. 여기서는 그대로 위임만 한다.
    static inline void _driveDataChunk(uint8_t chunkVal) __attribute__((always_inline)) {
        swp2p_driveDataChunk<PRESET>(chunkVal);
    }

    static inline uint8_t _readDataChunk() __attribute__((always_inline)) {
        return swp2p_readDataChunk<PRESET>();
    }

    static inline void _dataRelease() __attribute__((always_inline)) {
        swp2p_dataRelease<PRESET>();
    }

    static inline void _driveFrameChunk(uint8_t chunkVal) __attribute__((always_inline)) {
        swp2p_driveFrameChunk<PRESET>(chunkVal);
    }

    static inline void _frameDataRelease() __attribute__((always_inline)) {
        swp2p_frameDataRelease<PRESET>();
    }

    // 프리셋별 폭(WIDTH) 파생 상수도 SWP2PPreset.h의 SWP2PPresetTraits<PRESET>에서 그대로 가져다 쓴다.
    using Traits = SWP2PPresetTraits<PRESET>;
    static constexpr uint8_t DATA_WIDTH = Traits::DATA_WIDTH;
    static constexpr uint8_t ARB_CYCLES = Traits::ARB_CYCLES;
    static constexpr uint8_t DATA_MASK  = Traits::DATA_MASK;
    static constexpr uint8_t ARB_SHIFT  = Traits::ARB_SHIFT;
    static constexpr uint8_t TX_SHIFT   = Traits::TX_SHIFT;
    using FrameTraits = SWP2PFrameTraits<PRESET, SWP2P_MAX_BURST>;
    static constexpr uint8_t LEN_BITS = FrameTraits::LEN_BITS;
    static constexpr uint8_t LEN_CHUNKS = FrameTraits::LEN_CHUNKS;
    static constexpr uint8_t LEN_PADDING_BITS = FrameTraits::LEN_PADDING_BITS;
    static constexpr uint8_t ADDR_CHUNKS = FrameTraits::ADDR_CHUNKS;
    static constexpr uint8_t ADDR_PADDING_BITS = FrameTraits::ADDR_PADDING_BITS;

    // [버그4 수정] ACK 대기 타임아웃 (falling edge 기준 카운트). 수신측은 RX_DATA 마지막 청크를
    // 읽는 바로 그 순간(같은 클럭 사이클)에 곧장 PB0을 LOW로 구동하므로 이론상 거의 즉시 보여야
    // 하지만, 지터/노이즈 캔슬러 지연 여유를 감안해 몇 edge 정도는 기다려준다.
    static constexpr uint8_t ACK_WAIT_EDGES = 4;

    static inline void _finishRxAddress(uint16_t wireHeader) {
        uint16_t header = wireHeader >> ADDR_PADDING_BITS;
        uint8_t destId = (header >> 7) & SWP2P_NODE_MASK;
        bool isBurst = (header & 0x01) != 0;
        bool addressed = destId == _nodeId || destId == SWP2P_BROADCAST;

        _clkGoFast();
        if (addressed) SET_FLAG(FLAG_IS_MY_PACKET);
        else CLR_FLAG(FLAG_IS_MY_PACKET);
        if (!addressed && !_isClkMaster) {
            _rxState = 0;
            return;
        }
        _rxSrcByte = (header >> 1) & SWP2P_NODE_MASK;
#if SWP2P_ROUND_ROBIN
        _lastSenderId = _rxSrcByte;
#endif
        _rxDataByte = 0;
        _rxByteIdx = 0;
        if (isBurst) {
            _rxChunkCount = LEN_CHUNKS;
            _rxState = 3; // RX_LEN
        } else {
            _rxLen = 1;
            _rxChunkCount = ARB_CYCLES;
            _rxState = addressed ? 4 : 6;
        }
    }

    // ============================================================
    // CLK 엣지 핸들러 (INT0)
    // ============================================================
    static void _onClkEdge() {
        // [PIND] & [0000 0100] => [0000 0x00] (CLK=PD2 하이 상태 확인)
        bool clkHigh = (PIND & (1 << PD2)) != 0; // PD2, bit 2: 0000 0100

        if (clkHigh) {
            // --- 슬롯 중재: TX_PENDING(1) 노드가 자기 슬롯에 도달하면 버스를 점유 ---
            if (_txState == 1) { // TX_PENDING
                if (_slotCounter == _nodeId) {
                    // BUSY(PD3)가 아직 HIGH(idle)인지 확인
                    if ((PIND & (1 << PD3)) != 0) { // PD3, bit 3: 0000 1000
                        // 버스 점유: BUSY를 LOW로 구동 (open-drain assert)
                        PORTD &= ~(1 << PD3); // 마스크 PD3, bit 3: 0000 1000 (D3 LOW)
                        DDRD |= (1 << PD3);     // 마스크 PD3, bit 3: 0000 1000 (D3 BUSY 출력 켜서 라인 점유)
                        SET_FLAG(FLAG_IS_BUSY); // 송신 노드 자신도 즉시 BUSY 플래그 세팅
#if SWP2P_ROUND_ROBIN
                        _lastSenderId = _nodeId;
#endif

                        // 헤더 조립: DEST(6) + SRC(6) + BURST(1) = 13비트, MSB first
                        uint16_t header =
                            ((uint16_t)(_txDestId & SWP2P_NODE_MASK) << 7) |
                            ((uint16_t)_nodeId << 1) |
                            ((_txDestId & SWP2P_BURST_BIT) ? 1 : 0);
                        uint16_t headerReg = header << (16 - FrameTraits::ADDR_BITS);

                        // 첫 청크를 push-pull로 구동
                        uint8_t chunk = (headerReg >> ARB_SHIFT) & DATA_MASK;
                        headerReg <<= DATA_WIDTH;
                        _driveFrameChunk(chunk); // push-pull 구동!
                        _txArbReg = headerReg;   // 남은 헤더 비트 보관
                        _arbChunkCount = ADDR_CHUNKS - 1;
                        _txState = 2; // TX_HEADER
                        return;
                    }
                    // else: BUSY가 이미 LOW → 다른 노드가 먼저 점유, PENDING 유지하고 다음 idle 대기
                }
            }

            // 버스가 idle이고 송신 중이 아닐 때만 슬롯 카운터 증가
            if ((PIND & (1 << PD3)) != 0 && _txState < 2) {
                _slotCounter++;
                if (_slotCounter >= SWP2P_MAX_SLOTS) _slotCounter = 0;
            }

            // --- TX_HEADER(2): 헤더를 push-pull로 계속 전송 ---
            if (_txState == 2) {
                uint8_t chunkCnt = _arbChunkCount;
                if (chunkCnt == 0) {
                    // 헤더 전송 완료 → LEN 또는 DATA로 전이
                    if (_txLen > 1) {
                        _txDataReg = (_txLen - 2) << (8 - LEN_BITS);
                        _txDataChunkCount = LEN_CHUNKS;
                        // [시프트 & DATA_MASK] MSB 비트들을 하위로 내린 후 2^N-1 마스크로 필요 크기만 추출
                        uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                        _txDataReg <<= DATA_WIDTH;
                        _clkGoFast(); // HEADER 끝, LEN부터는 페이로드 구간으로 취급해 가속
                        _driveFrameChunk(chunk);
                        _txDataChunkCount--;
                        _txIdx = 0;
                        _txState = 3; // TX_LEN
                    } else {
                        _txDataReg = _txData;
                        _txDataChunkCount = ARB_CYCLES;
                        uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK; // 2^N-1 마스킹 추출
                        _txDataReg <<= DATA_WIDTH;
                        _clkGoFast(); // HEADER 끝, 곧바로 DATA로 진입하므로 여기서 가속
                        _driveFrameChunk(chunk);
                        _txDataChunkCount--;
                        _txState = 4; // TX_DATA
                    }
                } else {
                    // 헤더 남은 청크를 push-pull로 계속 구동
                    uint16_t headerReg = _txArbReg;
                    uint8_t chunk = (headerReg >> ARB_SHIFT) & DATA_MASK; // 2^N-1 마스킹 추출
                    headerReg <<= DATA_WIDTH;
                    _driveFrameChunk(chunk); // push-pull 구동 (open-drain이 아님!)
                    _txArbReg = headerReg;
                    _arbChunkCount = chunkCnt - 1;
                }
            } else if (_txState == 3) { // TX_LEN
                if (_txDataChunkCount == 0) {
                    _txDataReg = _txBuffer[0];
                    _txDataChunkCount = ARB_CYCLES;
                    uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK; // 2^N-1 마스킹 추출
                    _txDataReg <<= DATA_WIDTH;
                    _driveFrameChunk(chunk);
                    _txDataChunkCount--;
                    _txState = 4; // TX_DATA
                } else {
                    uint8_t dataReg = _txDataReg;
                    uint8_t chunk = (dataReg >> TX_SHIFT) & DATA_MASK; // 2^N-1 마스킹 추출
                    dataReg <<= DATA_WIDTH;
                    _driveFrameChunk(chunk);
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
                        _driveFrameChunk(chunk);
                        _txDataChunkCount--;
                    } else {
                        // [버그4 수정] 원래는 DATA 다 보내자마자 바로 BUSY를 풀고 "성공"으로 간주했음
                        // (상대가 실제로 받았는지 전혀 확인 안 함). 데이터 라인은 지금 놓아주되,
                        // BUSY(D3)는 계속 잡고 ACK 대기 상태(7)로 넘어가서 PB0(ACK)이 LOW로
                        // 내려오는지 확인한다.
                        _frameDataRelease();
                        _clkGoSlow(); // DATA 끝 -> ACK 대기부터는 다시 저속으로 복귀
                        _ackWaitCount = ACK_WAIT_EDGES;
                        _txState = 7; // TX_WAIT_ACK
                    }
                } else {
                    uint8_t dataReg = _txDataReg;
                    uint8_t chunk = (dataReg >> TX_SHIFT) & DATA_MASK; // 2^N-1 마스킹 추출
                    dataReg <<= DATA_WIDTH;
                    _driveFrameChunk(chunk);
                    _txDataReg = dataReg;
                    _txDataChunkCount--;
                }
            }
        } else {
            // ---- CLK 하강 에지 ----

            if (_txState == 7) { // [버그4 수정] TX_WAIT_ACK negedge: ACK(PB0=D8) 확인
                bool acked = (PINB & (1 << PB0)) == 0; // 마스크 PB0, bit 0: 0000 0001; 수신측이 LOW로 구동했는지
                if (acked) {
                    CLR_FLAG(FLAG_ACK_FAILED);
                    DDRD &= ~(1 << PD3); // 마스크 PD3, bit 3: 0000 1000; BUSY 해제
                    CLR_FLAG(FLAG_IS_SENDING);
                    _txState = 0;
                } else if (_ackWaitCount == 0) {
                    // 타임아웃: ACK 못 받았지만 버스는 풀어줘야 다른 노드가 계속 쓸 수 있음
                    SET_FLAG(FLAG_ACK_FAILED);
                    DDRD &= ~(1 << PD3); // 마스크 PD3, bit 3: 0000 1000
                    CLR_FLAG(FLAG_IS_SENDING);
                    _txState = 0;
                } else {
                    _ackWaitCount--;
                }
                return;
            }
            // 송신 중(HEADER/LEN/DATA)이면 하강 에지에서 할 일 없음 (충돌 검증 불필요)
            // TX_PENDING(1)인 노드는 송신자가 아니므로 다른 노드의 프레임을 수신해야 함
            if (_txState >= 2) return;

            uint8_t rxState = _rxState;

            if (rxState == 4) { // RX_DATA
                uint8_t v = _readDataChunk();
                // [기존 수신 데이터 << DATA_WIDTH] | [새 수신 청크 v] => 바이트 조립
                uint8_t dataByte = (_rxDataByte << DATA_WIDTH) | v;
                uint8_t chunkCnt = _rxChunkCount - 1;
                if (chunkCnt == 0) {
                    _fifoPush(dataByte, _rxSrcByte);
                    _rxByteIdx++;
                    if (_rxByteIdx < _rxLen) {
                        _rxDataByte = 0;
                        _rxChunkCount = ARB_CYCLES;
                    } else {
                        _clkGoSlow(); // DATA 끝 -> ACK부터는 다시 저속으로 복귀
                        _rxState = 5; // RX_ACK
                    }
                } else {
                    _rxDataByte = dataByte;
                    _rxChunkCount = chunkCnt;
                }
            } else if (rxState == 1) { // RX packed DEST(6) + SRC(6) + BURST(1)
                uint16_t header = ((uint16_t)_rxAddrByte << 8) | _rxSrcByte;
                header = (header << DATA_WIDTH) | _readDataChunk();
                _rxChunkCount--;
                if (_rxChunkCount == 0) _finishRxAddress(header);
                else {
                    _rxAddrByte = header >> 8;
                    _rxSrcByte = header;
                    uint8_t receivedBits = (ADDR_CHUNKS - _rxChunkCount) * DATA_WIDTH;
                    if (receivedBits >= 6) {
                        uint8_t destId = header >> (receivedBits - 6);
                        if (destId != _nodeId && destId != SWP2P_BROADCAST && !_isClkMaster) _rxState = 0;
                    }
                }
            } else if (rxState == 3) { // RX_LEN
                uint8_t v = _readDataChunk();
                _rxDataByte = (_rxDataByte << DATA_WIDTH) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    _rxLen = (_rxDataByte >> LEN_PADDING_BITS) + 2;
                    if (_rxLen > SWP2P_MAX_BURST) {
                        _rxByteIdx = 0;
                        _rxChunkCount = ARB_CYCLES;
                        _rxState = _isClkMaster ? 6 : 0;
                        return;
                    }
                    _rxByteIdx = 0;
                    _rxDataByte = 0;
                    _rxChunkCount = ARB_CYCLES;
                    _rxState = GET_FLAG(FLAG_IS_MY_PACKET) ? 4 : 6;
                }
            } else if (rxState == 6) { // RX_DISCARD: clock-master observes frame end without sampling data
                uint8_t chunkCnt = _rxChunkCount - 1;
                if (chunkCnt == 0) {
                    _rxByteIdx++;
                    if (_rxByteIdx < _rxLen) _rxChunkCount = ARB_CYCLES;
                    else {
                        _clkGoSlow();
                        _rxState = 0;
                    }
                } else {
                    _rxChunkCount = chunkCnt;
                }
            } else if (rxState == 5) { // RX_ACK
                // [버그4 전제 수정] 원래는 addressing 여부와 무관하게 무조건 ACK를 쏴서,
                // 모든 리슨 노드가 매 패킷마다 ACK를 구동했음(= ACK가 "내가 받았다"는 증거로 쓸모없었음).
                // 실제 목적지였던 노드만 ACK를 구동하도록 게이트.
                PORTB &= ~(1 << PB0); // 마스크 PB0, bit 0: 0000 0001
                DDRB |= (1 << PB0); // 마스크 PB0, bit 0: 0000 0001
                _rxState = 0;
            }
        }
    }

    static void _onBusyEdge() {
        // [PIND] & [0000 1000] => [0000 x000] (BUSY=PD3 상태 확인)
        bool idle = (PIND & (1 << PD3)) != 0; // PD3, bit 3: 0000 1000
        if (idle) {
            CLR_FLAG(FLAG_IS_BUSY);
            DDRB &= ~(1 << PB0); // 마스크 PB0, bit 0: 0000 0001 (PB0 입력/해제)
            _frameDataRelease(); // 방어적: 데이터 핀도 확실히 입력(High-Z)으로 해제
            // 방어적 backstop: HEADER/DATA 전환 훅을 어디선가 놓쳤더라도(버그, 예외 경로 등)
            // 버스가 idle로 돌아오는 시점엔 무조건 저속(HEADER 안전속도)으로 맞춰둔다.
            _clkGoSlow();
            _rxState = 0;
            _rxAddrByte = 0;
            _rxSrcByte = 0;
            _rxLen = 0;
            _rxByteIdx = 0;

            // 슬롯 카운터 리셋
#if SWP2P_ROUND_ROBIN
            _slotBase = (_lastSenderId + 1);
            if (_slotBase >= SWP2P_MAX_SLOTS) _slotBase = 0;
            _slotCounter = _slotBase;
#else
            _slotCounter = 0;
#endif

            // TX_PENDING(1) 상태면 그대로 유지 — 다음 idle 슬롯 순회에서 자기 차례를 기다림
        } else {
            SET_FLAG(FLAG_IS_BUSY);
            if (_txState <= 1) {
                _rxState = 1;
                _rxAddrByte = 0;
                _rxSrcByte = 0;
                _rxChunkCount = ADDR_CHUNKS;
            }
        }
    }

    static void _onAckCapture() {
        // [TCCR1B] ^= [0100 0000] => ICES1( bit6 ) 반전 (Rising Edge <-> Falling Edge 캡처 극성 토글)
        TCCR1B ^= (1 << ICES1); // ICES1, bit 6: 0100 0000
    }
};

#define SWP2P_BIND_ISRS(PRESET_TYPE) \
    ISR(INT0_vect)        { SWP2P<PRESET_TYPE>::_onClkEdge(); } \
    ISR(INT1_vect)        { SWP2P<PRESET_TYPE>::_onBusyEdge(); } \
    ISR(TIMER1_CAPT_vect) { SWP2P<PRESET_TYPE>::_onAckCapture(); }

#endif
