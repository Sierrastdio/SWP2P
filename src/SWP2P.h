#ifndef SWP2P_H
#define SWP2P_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define SWP2P_FIFO_DEPTH 4

#define FLAG_IS_SENDING   0
#define FLAG_IS_BUSY      1
#define FLAG_RX_CAPTURED  2
#define FLAG_IS_MY_PACKET 3

#define SET_FLAG(b) (GPIOR0 |= (1 << (b)))
#define CLR_FLAG(b) (GPIOR0 &= ~(1 << (b)))
#define GET_FLAG(b) (GPIOR0 & (1 << (b)))

enum DataPreset : uint8_t {
    PRESET_W1_D4 = 0, PRESET_W1_D5, PRESET_W1_D6, PRESET_W1_D7, PRESET_W1_D9, PRESET_W1_D10, PRESET_W1_A0,
    PRESET_W2_D4_D5, PRESET_W2_D6_D7, PRESET_W2_D9_D10, PRESET_W2_A0_A1,
    PRESET_W4_D4_D7, PRESET_W4_A0_A3, PRESET_W4_D9_D12,
    PRESET_W8_A0_D7
};

class SWP2PBase {
public:
    static uint8_t _nodeId;
    static volatile uint8_t _rxFifo[SWP2P_FIFO_DEPTH];
    static volatile uint8_t _rxHead;
    static volatile uint8_t _rxTail;
    static volatile uint8_t _rxCount;

    static volatile uint8_t _txState;
    static uint8_t _txDestId;
    static uint8_t _txData;
    static volatile uint16_t _txArbReg;
    static volatile uint8_t _txDataReg;
    static volatile uint8_t _arbChunkCount;
    static volatile uint8_t _txDataChunkCount;
    static volatile uint8_t _arbMyChunk;

    static volatile uint8_t _rxState;
    static volatile uint8_t _rxAddrByte;
    static volatile uint8_t _rxSrcByte;
    static volatile uint8_t _rxDataByte;
    static volatile uint8_t _rxChunkCount;

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
    SWP2P(uint8_t nodeId) {
        _nodeId = nodeId;
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

        // ISR 콜백 등록 (템플릿 함수를 정적 포인터에 바인딩)
        _isrClkCallback = _onClkEdge;
        _isrBusyCallback = _onBusyEdge;

        sei();
    }

    bool send(uint8_t destId, uint8_t data) {
        if (GET_FLAG(FLAG_IS_SENDING) || GET_FLAG(FLAG_IS_BUSY)) return false;
        _txDestId = destId;
        _txData = data;
        _txState = 1;
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
            if (_txState == 1) {
                PORTD &= ~(1 << PORTD3); DDRD |= (1 << DDD3);
                _txArbReg = ((uint16_t)_txDestId << 8) | _nodeId;
                _arbChunkCount = 2 * ARB_CYCLES;
                _txState = 2;

                _arbMyChunk = (_txArbReg >> ARB_SHIFT) & DATA_MASK;
                _txArbReg <<= DATA_WIDTH;
                _driveDataChunk(_arbMyChunk);
                _arbChunkCount--;
                return;
            }

            if (_txState == 2) {
                if (_arbChunkCount == 0) {
                    _txDataReg = _txData;
                    _txDataChunkCount = ARB_CYCLES;
                    uint8_t chunk = (_txDataReg >> TX_SHIFT) & DATA_MASK;
                    _txDataReg <<= DATA_WIDTH;
                    _driveDataChunk(chunk);
                    _txDataChunkCount--;
                    _txState = 3;
                } else {
                    _arbMyChunk = (_txArbReg >> ARB_SHIFT) & DATA_MASK;
                    _txArbReg <<= DATA_WIDTH;
                    _driveDataChunk(_arbMyChunk);
                    _arbChunkCount--;
                }
            } else if (_txState == 3) {
                if (_txDataChunkCount == 0) {
                    _dataRelease();
                    _txState = 5;
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
            if (_txState == 2) {
                uint8_t busVal = _readDataChunk();
                if ((_arbMyChunk & ~busVal) != 0) {
                    _dataRelease();
                    DDRD &= ~(1 << DDD3);
                    CLR_FLAG(FLAG_IS_SENDING);
                    _txState = 6;
                }
                return;
            }
            if (_txState != 0) return;

            if (_rxState == 1) {
                uint8_t v = _readDataChunk();
                _rxAddrByte = (_rxAddrByte << DATA_WIDTH) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    if (_rxAddrByte == _nodeId || _rxAddrByte == 0xFF) {
                        SET_FLAG(FLAG_IS_MY_PACKET);
                    } else {
                        CLR_FLAG(FLAG_IS_MY_PACKET);
                    }
                    _rxSrcByte = 0;
                    _rxChunkCount = ARB_CYCLES;
                    _rxState = 2;
                }
            } else if (_rxState == 2) {
                uint8_t v = _readDataChunk();
                _rxSrcByte = (_rxSrcByte << DATA_WIDTH) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    _rxDataByte = 0;
                    _rxChunkCount = ARB_CYCLES;
                    _rxState = 3;
                }
            } else if (_rxState == 3) {
                uint8_t v = _readDataChunk();
                _rxDataByte = (_rxDataByte << DATA_WIDTH) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    if (GET_FLAG(FLAG_IS_MY_PACKET) && !GET_FLAG(FLAG_RX_CAPTURED)) {
                        _fifoPush(_rxDataByte);
                        SET_FLAG(FLAG_RX_CAPTURED);
                        _rxState = 4;
                    } else {
                        _rxState = 0;
                    }
                }
            } else if (_rxState == 4) {
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
            _rxState = 0;
            _rxAddrByte = 0;
            _rxSrcByte = 0;

            if (_txState == 6) {
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
