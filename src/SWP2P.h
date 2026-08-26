#ifndef SWP2P_H
#define SWP2P_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define SWP2P_FIFO_DEPTH 4
#define SWP2P_MAX_BURST  16   // _txBuffer 크기. len-2를 8비트 레지스터에 실어 보내므로 이론상 257까지 가능하지만
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
#define FLAG_RX_IS_BURST  4   // 신규: 이번 수신 프레임이 burst인지 (RX_SRC 완료 시점에 결정되어 저장됨)

#define SET_FLAG(b) (GPIOR0 |= (1 << (b)))
#define CLR_FLAG(b) (GPIOR0 &= ~(1 << (b)))
#define GET_FLAG(b) (GPIOR0 & (1 << (b)))

enum DataPreset : uint8_t {
    PRESET_W1_D4 = 0, PRESET_W1_D5, PRESET_W1_D6, PRESET_W1_D7, PRESET_W1_D9, PRESET_W1_D10, PRESET_W1_A0,
    PRESET_W2_D4_D5, PRESET_W2_D6_D7, PRESET_W2_D9_D10, PRESET_W2_A0_A1,
    PRESET_W4_D4_D7, PRESET_W4_A0_A3, PRESET_W4_D9_D12,
    PRESET_W8_A0_D7
};

// ---- Tx/Rx 상태 번호 (원래 코드의 번호를 최대한 유지하고, LEN 단계만 새로 끼워 넣음) ----
// Tx: 0 IDLE, 1 PENDING, 2 ARB, 3 LEN(신규), 4 DATA(원래 3번이었음), 5 RELEASE/DONE(원래와 동일), 6 LOST
// Rx: 0 IDLE, 1 ADDR, 2 SRC, 3 LEN(신규), 4 DATA(원래 3번이었음), 5 ACK(원래 4번이었음)

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
    static volatile uint8_t _rxLen;        // 이번 프레임에서 받아야 할 총 바이트 수 (신규)
    static volatile uint8_t _rxByteIdx;    // 지금까지 받은 바이트 수 (신규)

    static void _fifoPush(uint8_t val);
    static void setupTimer1(unsigned long freq);
    static void stopTimer1();

    // ISR 브릿지 포인터 (템플릿 인스턴스 핸들러를 ISR에서 안전하게 호출하기 위함)
    static void (*_isrClkCallback)();
    static void (*_isrBusyCallback)();
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
        }

        EICRA &= ~((1 << ISC01) | (1 << ISC00));
        EICRA |= (1 << ISC00);
        EIMSK |= (1 << INT0);

        EICRA &= ~((1 << ISC11) | (1 << ISC10));
        EICRA |= (1 << ISC10);
        EIMSK |= (1 << INT1);

        PCICR |= (1 << PCIE0);
        PCMSK0 |= (1 << PCINT0);

        _isrClkCallback = _onClkEdge;
        _isrBusyCallback = _onBusyEdge;

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

    static void _onClkEdge() {
        bool clkHigh = (PIND & (1 << PIND2)) != 0;

        if (clkHigh) {
            if (_txState == 1) { // TX_PENDING
                PORTD &= ~(1 << PORTD3); DDRD |= (1 << DDD3);
                _txArbReg = ((uint16_t)_txDestId << 8) | _nodeId;
                _arbChunkCount = 2 * ARB_CYCLES;
                _txState = 2; // TX_ARB

                _arbMyChunk = (_txArbReg >> ARB_SHIFT) & DATA_MASK;
                _txArbReg <<= DATA_WIDTH;
                _driveDataChunk(_arbMyChunk);
                _arbChunkCount--;
                return;
            }

            if (_txState == 2) { // TX_ARB
                if (_arbChunkCount == 0) {
                    // 중재(주소 2바이트: dest+src) 완료 -> burst 여부에 따라 분기
                    if (_txLen > 1) {
                        // LEN 필드 전송 시작: len-2를 인코딩 (len은 2~SWP2P_MAX_BURST이므로 0..(MAX-2) 범위)
                        _txDataReg = _txLen - 2;
                        _txDataChunkCount = ARB_CYCLES;
                        uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                        _txDataReg <<= DATA_WIDTH;
                        _driveDataChunk(chunk);
                        _txDataChunkCount--;
                        _txIdx = 0;
                        _txState = 3; // TX_LEN
                    } else {
                        // 기존 단일 바이트 빠른 경로 - 완전히 그대로
                        _txDataReg = _txData;
                        _txDataChunkCount = ARB_CYCLES;
                        uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                        _txDataReg <<= DATA_WIDTH;
                        _driveDataChunk(chunk);
                        _txDataChunkCount--;
                        _txState = 4; // TX_DATA
                    }
                } else {
                    _arbMyChunk = (_txArbReg >> ARB_SHIFT) & DATA_MASK;
                    _txArbReg <<= DATA_WIDTH;
                    _driveDataChunk(_arbMyChunk);
                    _arbChunkCount--;
                }
            } else if (_txState == 3) { // TX_LEN (신규)
                if (_txDataChunkCount == 0) {
                    // LEN 필드 끝 -> 첫 데이터 바이트 시작
                    _txDataReg = _txBuffer[0];
                    _txDataChunkCount = ARB_CYCLES;
                    uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                    _txDataReg <<= DATA_WIDTH;
                    _driveDataChunk(chunk);
                    _txDataChunkCount--;
                    _txState = 4; // TX_DATA
                } else {
                    uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                    _txDataReg <<= DATA_WIDTH;
                    _driveDataChunk(chunk);
                    _txDataChunkCount--;
                }
            } else if (_txState == 4) { // TX_DATA (원래 3번)
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
                        _txState = 5; // TX_RELEASE/DONE (원래와 동일)
                    }
                } else {
                    uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                    _txDataReg <<= DATA_WIDTH;
                    _driveDataChunk(chunk);
                    _txDataChunkCount--;
                }
            } else if (_txState == 5) {
                DDRD &= ~(1 << DDD3);
                CLR_FLAG(FLAG_IS_SENDING);
                _txState = 0;
            }
        } else {
            if (_txState == 2) { // TX_ARB negedge: 되읽기 검증 (원래와 동일)
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

            if (_rxState == 1) { // RX_ADDR
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
            } else if (_rxState == 2) { // RX_SRC
                uint8_t v = _readDataChunk();
                _rxSrcByte = (_rxSrcByte << DATA_WIDTH) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    if (GET_FLAG(FLAG_RX_IS_BURST)) {
                        // LEN 필드 수신 시작 (_rxDataByte를 임시 누산기로 재사용)
                        _rxDataByte = 0;
                        _rxChunkCount = ARB_CYCLES;
                        _rxState = 3; // RX_LEN
                    } else {
                        // 기존 단일 바이트 경로 - 완전히 그대로
                        _rxDataByte = 0;
                        _rxChunkCount = ARB_CYCLES;
                        _rxLen = 1;
                        _rxByteIdx = 0;
                        _rxState = 4; // RX_DATA
                    }
                }
            } else if (_rxState == 3) { // RX_LEN (신규)
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
            } else if (_rxState == 4) { // RX_DATA (원래 3번)
                uint8_t v = _readDataChunk();
                _rxDataByte = (_rxDataByte << DATA_WIDTH) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    if (GET_FLAG(FLAG_IS_MY_PACKET)) {
                        _fifoPush(_rxDataByte);
                        SET_FLAG(FLAG_RX_CAPTURED);
                    }
                    _rxByteIdx++;
                    if (_rxByteIdx < _rxLen) {
                        // burst의 다음 바이트를 이어서 수신 (RX_DATA에 계속 머무름)
                        _rxDataByte = 0;
                        _rxChunkCount = ARB_CYCLES;
                    } else {
                        _rxState = 5; // RX_ACK (원래 4번)
                    }
                }
            } else if (_rxState == 5) { // RX_ACK
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

            if (_txState == 6) { // TX_LOST -> 재시도 (기존과 동일. _txLen/_txBuffer는 그대로 살아있어 burst 재시도도 안전)
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
};

#endif
