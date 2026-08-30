#ifndef SWP2P_H
#define SWP2P_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "SWP2PBuffer.h"

#define SWP2P_FIFO_DEPTH 16
#define SWP2P_MAX_BURST   16   // _txBuffer 크기. len-2를 8비트 레지스터에 실어 보내므로 이론상 257까지 가능하지만
                              // RAM 절약 위해 우선 16으로 제한 (필요시 늘리면 됨)

// ---- 주소 인코딩 ----
// destId 바이트의 MSB(bit7)를 "burst 프레임 여부" 플래그로 사용.
// 그 결과 실제 NODE_ID는 0~126(7비트)만 쓸 수 있고, 브로드캐스트 주소도 0x7F로 바뀐다.
// (노드 수가 10개 안팎이라 7비트 주소공간으로 충분하다는 전제)
#define SWP2P_NODE_MASK   0x7F
#define SWP2P_BURST_BIT   0x80
#define SWP2P_BROADCAST   0x7F

#define FLAG_IS_SENDING   0
#define FLAG_IS_BUSY      1
#define FLAG_RX_CAPTURED  2
#define FLAG_IS_MY_PACKET 3
#define FLAG_RX_IS_BURST  4   // 이번 수신 프레임이 burst인지 (RX_SRC 완료 시점에 결정되어 저장됨)

#define SET_FLAG(b) (GPIOR0 |= (1 << (b)))
#define CLR_FLAG(b) (GPIOR0 &= ~(1 << (b)))
#define GET_FLAG(b) (GPIOR0 & (1 << (b)))

enum DataPreset : uint8_t {
    PRESET_W1_D4 = 0, PRESET_W1_D5, PRESET_W1_D6, PRESET_W1_D7, PRESET_W1_D9, PRESET_W1_D10, PRESET_W1_A0,
    PRESET_W2_D4_D5, PRESET_W2_D6_D7, PRESET_W2_D9_D10, PRESET_W2_A0_A1,
    PRESET_W4_D4_D7, PRESET_W4_A0_A3, PRESET_W4_D9_D12,
    PRESET_W8_A0_D7
};

// ---- Tx/Rx 상태 번호 ----
// Tx: 0 IDLE, 1 PENDING, 2 ARB, 3 LEN, 4 DATA, 5 RELEASE/DONE, 6 LOST
// Rx: 0 IDLE, 1 ADDR, 2 SRC, 3 LEN, 4 DATA, 5 ACK

class SWP2PBase {
public:
    static uint8_t _nodeId;
    static volatile uint8_t _rxFifo[SWP2P_FIFO_DEPTH];
    static volatile uint8_t _rxHead;
    static volatile uint8_t _rxTail;
    static volatile uint8_t _rxCount;

    static volatile uint8_t _txState;
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

    static volatile uint8_t _rxState;
    static volatile uint8_t _rxAddrByte;
    static volatile uint8_t _rxSrcByte;
    static volatile uint8_t _rxDataByte;   // 데이터 바이트 수신용 + LEN 필드 수신에도 재사용
    static volatile uint8_t _rxChunkCount;
    static volatile uint8_t _rxLen;        // 이번 프레임에서 받아야 할 총 바이트 수
    static volatile uint8_t _rxByteIdx;    // 지금까지 받은 바이트 수

    static void _fifoPush(uint8_t val);
    static void setupTimer1(unsigned long freq);
    static void stopTimer1();

    // 주의: 함수 포인터 기반 ISR 브릿지는 제거됨.
    // ISR은 파일 하단의 SWP2P_BIND_ISRS(PRESET) 매크로로 컴파일타임에 직접 바인딩한다.
    // (icall/ret 오버헤드 및 별도 프롤로그/에필로그 생성을 없애 ISR 실행시간을 줄이기 위함)
};

template <DataPreset PRESET>
class SWP2P : public SWP2PBase {
public:
    static constexpr uint8_t BROADCAST = SWP2P_BROADCAST;

    SWP2P(uint8_t nodeId) {
        // 상위 비트를 burst 플래그로 쓰기로 했으므로 NODE_ID는 7비트로 강제 마스킹.
        // (0x7F=127은 브로드캐스트 전용으로 예약되어 있으니 실질적으로 0~126 사용 가능)
        _nodeId = nodeId & SWP2P_NODE_MASK;
    }

    void begin(bool clkIsOutput, unsigned long clkFreq = 100000UL) {
        GPIOR0 = 0;

        DDRD &= ~(1 << DDD2);
        PORTD &= ~(1 << PORTD2);

        PORTD &= ~(1 << PORTD3);
        DDRD &= ~(1 << DDD3);
        PORTB &= ~(1 << PORTB0);
        DDRB &= ~(1 << DDB0);

        _dataRelease();

        if (clkIsOutput) {
            DDRB |= (1 << DDB1);
            setupTimer1(clkFreq);
        } else {
            // ICU(Input Capture Unit)가 동작하려면 Timer1이 최소한 프리러닝 상태여야 함
            // (엣지 검출 자체는 프리스케일러와 무관하지만, 안전하게 CS10만 켜둔다)
            TCCR1A = 0;
            TCCR1B = (1 << CS10);
        }

        // ---- ACK_N(D8 = PB0 = ICP1) 엣지 감지: PCINT 폴링 대신 Input Capture Unit 사용 ----
        // 기존 코드는 PCINT0로 8개 핀을 통째로 스캔하는 방식이었고 ISR 본체도 비어있었다.
        // ICP1은 마침 ACK_N 핀(PB0)과 정확히 겹치므로, 하드웨어 엣지검출+노이즈캔슬러를
        // 그대로 활용해 더 가볍고 신뢰성 높은 방식으로 대체한다.
        TCCR1B |= (1 << ICNC1);  // 노이즈 캔슬러 on
        TCCR1B |= (1 << ICES1);  // 우선 rising edge부터 포착 (엣지가 잡힐 때마다 ISR에서 극성 토글)
        TIMSK1 |= (1 << ICIE1);  // Input Capture 인터럽트 활성화

        EICRA &= ~((1 << ISC01) | (1 << ISC00));
        EICRA |= (1 << ISC00);   // INT0: CLK, any logical change
        EIMSK |= (1 << INT0);

        EICRA &= ~((1 << ISC11) | (1 << ISC10));
        EICRA |= (1 << ISC10);   // INT1: BUSY_N, any logical change
        EIMSK |= (1 << INT1);

        sei();
    }

    // 단일 바이트 전송 - 기존과 완전히 동일한 빠른 경로 (burst 플래그=0, 길이 필드 없음)
    bool send(uint8_t destId, uint8_t data) {
        return sendBurst(destId, &data, 1);
    }

    // 버스트 전송 - len>1이면 destId 상위비트에 burst 플래그가 세팅되고, LEN 필드가 추가로 실린다.
    bool sendBurst(uint8_t destId, const uint8_t* buf, uint8_t len) {
        if (len == 0 || len > SWP2P_MAX_BURST) return false;
        if (GET_FLAG(FLAG_IS_SENDING) || GET_FLAG(FLAG_IS_BUSY)) return false;

        uint8_t d = destId & SWP2P_NODE_MASK;
        if (len > 1) d |= SWP2P_BURST_BIT;
        _txDestId = d;
        _txLen = len;
        _txIdx = 0;

        if (len == 1) {
            _txData = buf[0]; // 기존 경로 그대로
        } else {
            for (uint8_t i = 0; i < len; i++) _txBuffer[i] = buf[i];
        }

        _txState = 1; // TX_PENDING
        SET_FLAG(FLAG_IS_SENDING);
        return true;
    }

    // ---- 사용자 친화 buffer API ----
    template <uint8_t CAP>
    bool buff(SWP2PBuffer<CAP>& buf, bool bit) {
        return buf.pushBit(bit);
    }
    template <uint8_t CAP>
    bool buff(SWP2PBuffer<CAP>& buf, uint8_t byteVal, bool /*asByte*/) {
        return buf.pushByte(byteVal);
    }

    template <uint8_t CAP>
    void buffFree(SWP2PBuffer<CAP>& buf) { buf.clearAll(); }
    template <uint8_t CAP>
    void buffFree(SWP2PBuffer<CAP>& buf, uint8_t idx) { buf.clearBuffer(idx); }

    template <uint8_t CAP>
    bool sendBurst(uint8_t destId, SWP2PBuffer<CAP>& buf) {
        static_assert(CAP <= SWP2P_MAX_BURST,
            "SWP2PBuffer CAP exceeds SWP2P_MAX_BURST - reduce CAP or increase SWP2P_MAX_BURST");
        return sendBurst(destId, buf.data, buf.len());
    }
    template <uint8_t CAP>
    bool sendBurst(uint8_t destId, SWP2PBuffer<CAP>& buf, uint8_t offset, uint8_t len) {
        static_assert(CAP <= SWP2P_MAX_BURST,
            "SWP2PBuffer CAP exceeds SWP2P_MAX_BURST - reduce CAP or increase SWP2P_MAX_BURST");
        if ((uint16_t)offset + len > buf.len()) return false;
        return sendBurst(destId, buf.data + offset, len);
    }

    bool available() { return _rxCount > 0; }

    uint8_t read() {
        if (_rxCount == 0) return 0;
        uint8_t val = _rxFifo[_rxTail];
        _rxTail = (_rxTail + 1) & (SWP2P_FIFO_DEPTH - 1);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            _rxCount--;
        }
        return val;
    }

    uint8_t readBytes(uint8_t* outBuf, uint8_t maxLen) {
        uint8_t count = 0;
        while (count < maxLen && available()) outBuf[count++] = read();
        return count;
    }

    uint8_t peek() {
        if (_rxCount == 0) return 0;
        return _rxFifo[_rxTail];
    }

    void flush() {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            _rxHead = 0; _rxTail = 0; _rxCount = 0;
        }
    }

    bool isSending() const { return GET_FLAG(FLAG_IS_SENDING); }
    bool isBusy() const { return GET_FLAG(FLAG_IS_BUSY); }

    // Compile-time Dispatching (Switch-case 완전 제거, 컴파일러가 최적화하여 100% 인라인 전개)
    static inline void _driveDataChunk(uint8_t chunkVal) __attribute__((always_inline)) {
        if constexpr (PRESET == PRESET_W1_D4) { if (!(chunkVal & 1)) DDRD |= (1 << DDD4); else DDRD &= ~(1 << DDD4); }
        else if constexpr (PRESET == PRESET_W1_D5) { if (!(chunkVal & 1)) DDRD |= (1 << DDD5); else DDRD &= ~(1 << DDD5); }
        else if constexpr (PRESET == PRESET_W1_D6) { if (!(chunkVal & 1)) DDRD |= (1 << DDD6); else DDRD &= ~(1 << DDD6); }
        else if constexpr (PRESET == PRESET_W1_D7) { if (!(chunkVal & 1)) DDRD |= (1 << DDD7); else DDRD &= ~(1 << DDD7); }
        else if constexpr (PRESET == PRESET_W1_D9) { if (!(chunkVal & 1)) DDRB |= (1 << DDB1); else DDRB &= ~(1 << DDB1); }
        else if constexpr (PRESET == PRESET_W1_D10){ if (!(chunkVal & 1)) DDRB |= (1 << DDB2); else DDRB &= ~(1 << DDB2); }
        else if constexpr (PRESET == PRESET_W1_A0) { if (!(chunkVal & 1)) DDRC |= (1 << DDC0); else DDRC &= ~(1 << DDC0); }
        else if constexpr (PRESET == PRESET_W2_D4_D5) { DDRD = (DDRD & ~0x30) | ((~chunkVal & 0x03) << 4); }
        else if constexpr (PRESET == PRESET_W2_D6_D7) { DDRD = (DDRD & ~0xC0) | ((~chunkVal & 0x03) << 6); }
        else if constexpr (PRESET == PRESET_W2_D9_D10){ DDRB = (DDRB & ~0x06) | ((~chunkVal & 0x03) << 1); }
        else if constexpr (PRESET == PRESET_W2_A0_A1) { DDRC = (DDRC & ~0x03) | (~chunkVal & 0x03); }
        else if constexpr (PRESET == PRESET_W4_D4_D7) { DDRD = (DDRD & ~0xF0) | ((~chunkVal & 0x0F) << 4); }
        else if constexpr (PRESET == PRESET_W4_A0_A3) { DDRC = (DDRC & ~0x0F) | (~chunkVal & 0x0F); }
        else if constexpr (PRESET == PRESET_W8_A0_D7) {
            DDRC = (DDRC & ~0x0F) | (~chunkVal & 0x0F);
            DDRD = (DDRD & ~0xF0) | ((~chunkVal & 0xF0));
        }
    }

    static inline uint8_t _readDataChunk() __attribute__((always_inline)) {
        uint8_t val = 0;
        if constexpr (PRESET == PRESET_W1_D4)  val = (PIND & (1 << PIND4)) ? 1 : 0;
        else if constexpr (PRESET == PRESET_W1_D5)  val = (PIND & (1 << PIND5)) ? 1 : 0;
        else if constexpr (PRESET == PRESET_W1_D6)  val = (PIND & (1 << PIND6)) ? 1 : 0;
        else if constexpr (PRESET == PRESET_W1_D7)  val = (PIND & (1 << PIND7)) ? 1 : 0;
        else if constexpr (PRESET == PRESET_W1_D9)  val = (PINB & (1 << PINB1)) ? 1 : 0;
        else if constexpr (PRESET == PRESET_W1_D10) val = (PINB & (1 << PINB2)) ? 1 : 0;
        else if constexpr (PRESET == PRESET_W1_A0)  val = (PINC & (1 << PINC0)) ? 1 : 0;
        else if constexpr (PRESET == PRESET_W2_D4_D5) val = (PIND & 0x30) >> 4;
        else if constexpr (PRESET == PRESET_W2_D6_D7) val = (PIND & 0xC0) >> 6;
        else if constexpr (PRESET == PRESET_W2_D9_D10)val = (PINB & 0x06) >> 1;
        else if constexpr (PRESET == PRESET_W2_A0_A1) val = (PINC & 0x03);
        else if constexpr (PRESET == PRESET_W4_D4_D7) val = (PIND & 0xF0) >> 4;
        else if constexpr (PRESET == PRESET_W4_A0_A3) val = (PINC & 0x0F);
        else if constexpr (PRESET == PRESET_W8_A0_D7) val = (PINC & 0x0F) | (PIND & 0xF0);
        return val;
    }

    static inline void _dataRelease() __attribute__((always_inline)) {
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
        else if constexpr (PRESET == PRESET_W8_A0_D7) { DDRC &= ~0x0F; DDRD &= ~0xF0; }
    }

    static constexpr uint8_t DATA_WIDTH =
        (PRESET >= PRESET_W8_A0_D7) ? 8 :
        (PRESET >= PRESET_W4_D4_D7) ? 4 :
        (PRESET >= PRESET_W2_D4_D5) ? 2 : 1;

    static constexpr uint8_t ARB_CYCLES = (8 + DATA_WIDTH - 1) / DATA_WIDTH;
    static constexpr uint8_t DATA_MASK = (1 << DATA_WIDTH) - 1;
    static constexpr uint8_t ARB_SHIFT = 16 - DATA_WIDTH;
    static constexpr uint8_t TX_SHIFT = 8 - DATA_WIDTH;

    // ============================================================
    // CLK 엣지 핸들러 (INT0) - volatile 캐싱 최적화 적용
    // 반복 빈도가 가장 높은 두 구간(ARB 진행, DATA 진행)에서 volatile
    // 멤버를 여러 번 읽고/쓰는 대신 지역 변수에 한 번 캐싱해서 계산 후
    // 한 번만 써준다. 그 외(프레임당 1회만 도는 상태 전이)는 원본 그대로
    // 두어 불필요한 리스크를 줄였다.
    // ============================================================
    static void _onClkEdge() {
        bool clkHigh = (PIND & (1 << PIND2)) != 0;

        if (clkHigh) {
            if (_txState == 1) { // TX_PENDING
                PORTD &= ~(1 << PORTD3); DDRD |= (1 << DDD3);
                uint16_t arbReg = ((uint16_t)_txDestId << 8) | _nodeId;
                uint8_t myChunk = (arbReg >> ARB_SHIFT) & DATA_MASK;
                arbReg <<= DATA_WIDTH;
                _driveDataChunk(myChunk);
                _txArbReg = arbReg;
                _arbMyChunk = myChunk;
                _arbChunkCount = (2 * ARB_CYCLES) - 1;
                _txState = 2; // TX_ARB
                return;
            }

            if (_txState == 2) { // TX_ARB
                uint8_t chunkCnt = _arbChunkCount;
                if (chunkCnt == 0) {
                    // 중재(주소 2바이트: dest+src) 완료 -> burst 여부에 따라 분기
                    if (_txLen > 1) {
                        _txDataReg = _txLen - 2; // LEN 필드 (len-2 인코딩)
                        _txDataChunkCount = ARB_CYCLES;
                        uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                        _txDataReg <<= DATA_WIDTH;
                        _driveDataChunk(chunk);
                        _txDataChunkCount--;
                        _txIdx = 0;
                        _txState = 3; // TX_LEN
                    } else {
                        _txDataReg = _txData; // 기존 단일 바이트 빠른 경로
                        _txDataChunkCount = ARB_CYCLES;
                        uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                        _txDataReg <<= DATA_WIDTH;
                        _driveDataChunk(chunk);
                        _txDataChunkCount--;
                        _txState = 4; // TX_DATA
                    }
                } else {
                    // ---- 핫패스: 캐싱 적용 ----
                    uint16_t arbReg = _txArbReg;
                    uint8_t myChunk = (arbReg >> ARB_SHIFT) & DATA_MASK;
                    arbReg <<= DATA_WIDTH;
                    _driveDataChunk(myChunk);
                    _txArbReg = arbReg;
                    _arbMyChunk = myChunk;
                    _arbChunkCount = chunkCnt - 1;
                }
            } else if (_txState == 3) { // TX_LEN
                if (_txDataChunkCount == 0) {
                    _txDataReg = _txBuffer[0];
                    _txDataChunkCount = ARB_CYCLES;
                    uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                    _txDataReg <<= DATA_WIDTH;
                    _driveDataChunk(chunk);
                    _txDataChunkCount--;
                    _txState = 4; // TX_DATA
                } else {
                    uint8_t dataReg = _txDataReg;
                    uint8_t chunk = (dataReg >> TX_SHIFT) & DATA_MASK;
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
                        uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                        _txDataReg <<= DATA_WIDTH;
                        _driveDataChunk(chunk);
                        _txDataChunkCount--;
                    } else {
                        _dataRelease();
                        _txState = 5; // TX_RELEASE/DONE
                    }
                } else {
                    // ---- 핫패스: 캐싱 적용 ----
                    uint8_t dataReg = _txDataReg;
                    uint8_t chunk = (dataReg >> TX_SHIFT) & DATA_MASK;
                    dataReg <<= DATA_WIDTH;
                    _driveDataChunk(chunk);
                    _txDataReg = dataReg;
                    _txDataChunkCount--;
                }
            } else if (_txState == 5) {
                DDRD &= ~(1 << DDD3);
                CLR_FLAG(FLAG_IS_SENDING);
                _txState = 0;
            }
        } else {
            if (_txState == 2) { // TX_ARB negedge: 되읽기 검증
                uint8_t busVal = _readDataChunk();
                if ((_arbMyChunk & ~busVal) != 0) {
                    _dataRelease();
                    DDRD &= ~(1 << DDD3);
                    CLR_FLAG(FLAG_IS_SENDING);
                    _txState = 6; // TX_LOST
                }
                return;
            }
            if (_txState != 0) return;

            // ---- RX 상태 분기를 빈도순으로 재배치: DATA가 가장 오래 머무는 상태이므로 맨 앞으로 ----
            uint8_t rxState = _rxState;

            if (rxState == 4) { // RX_DATA - 핫패스, 캐싱 적용
                uint8_t v = _readDataChunk();
                uint8_t dataByte = (_rxDataByte << DATA_WIDTH) | v;
                uint8_t chunkCnt = _rxChunkCount - 1;
                if (chunkCnt == 0) {
                    if (GET_FLAG(FLAG_IS_MY_PACKET)) {
                        _fifoPush(dataByte);
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
                    uint8_t addr7 = _rxAddrByte & SWP2P_NODE_MASK;
                    bool isBurst = (_rxAddrByte & SWP2P_BURST_BIT) != 0;
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
                    _rxLen = _rxDataByte + 2; // 인코딩(len-2) 복원
                    _rxByteIdx = 0;
                    _rxDataByte = 0;
                    _rxChunkCount = ARB_CYCLES;
                    _rxState = 4; // RX_DATA
                }
            } else if (rxState == 5) { // RX_ACK
                PORTB &= ~(1 << PORTB0); DDRB |= (1 << DDB0);
                _rxState = 0;
            }
        }
    }

    static void _onBusyEdge() {
        bool idle = (PIND & (1 << PIND3)) != 0;
        if (idle) {
            CLR_FLAG(FLAG_IS_BUSY);
            DDRB &= ~(1 << DDB0);
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

    // ============================================================
    // ACK_N(D8=PB0=ICP1) 엣지 핸들러 - Input Capture Unit(TIMER1_CAPT)
    // 기존 PCINT0 기반 스캔(본체는 비어있었음)을 대체.
    // 현재 프로토콜은 별도 타임스탬프를 쓰지 않으므로 엣지 발생 자체만
    // 소비하고 다음 엣지(반대 극성)를 잡도록 극성만 토글한다.
    // 향후 정밀 타이밍 기반 흐름제어가 필요해지면 ICR1 레지스터 값을
    // 여기서 함께 읽어 활용하면 된다.
    // ============================================================
    static void _onAckCapture() {
        TCCR1B ^= (1 << ICES1);
    }
};

// ============================================================
// ISR 직접 바인딩 매크로
// 함수 포인터 간접호출(icall)과 그로 인한 별도 함수 프롤로그/에필로그
// 생성을 없애기 위해, 실제 사용하는 PRESET으로 ISR을 컴파일타임에
// 직접 바인딩한다. .ino 또는 .cpp 파일에서 노드를 선언한 뒤
// 전역 스코프에서 딱 한 번 호출한다:
//
//   SWP2P<PRESET_W1_D4> node(1);
//   SWP2P_BIND_ISRS(PRESET_W1_D4);   // <- 노드의 PRESET과 반드시 동일해야 함
//
// 한 펌웨어에 SWP2P 노드를 하나만 쓰는 일반적인 경우를 전제로 한다.
// ============================================================
#define SWP2P_BIND_ISRS(PRESET_TYPE) \
    ISR(INT0_vect)        { SWP2P<PRESET_TYPE>::_onClkEdge(); } \
    ISR(INT1_vect)        { SWP2P<PRESET_TYPE>::_onBusyEdge(); } \
    ISR(TIMER1_CAPT_vect) { SWP2P<PRESET_TYPE>::_onAckCapture(); }

#endif
