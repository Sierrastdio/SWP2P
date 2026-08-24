#ifndef SWP2P_H
#define SWP2P_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define SWP2P_FIFO_DEPTH 4

// GPIOR0 Hardware Bit Flags (1-cycle operations)
#define FLAG_IS_SENDING   0
#define FLAG_IS_BUSY      1
#define FLAG_RX_CAPTURED  2
#define FLAG_IS_MY_PACKET 3

#define SET_FLAG(b) (GPIOR0 |= (1 << (b)))
#define CLR_FLAG(b) (GPIOR0 &= ~(1 << (b)))
#define GET_FLAG(b) (GPIOR0 & (1 << (b)))

enum DataPreset : uint8_t {
    // Width = 1
    PRESET_W1_D4 = 0,
    PRESET_W1_D5,
    PRESET_W1_D6,
    PRESET_W1_D7,
    PRESET_W1_D9,
    PRESET_W1_D10,
    PRESET_W1_A0,

    // Width = 2 (MSB -> LSB)
    PRESET_W2_D4_D5,
    PRESET_W2_D6_D7,
    PRESET_W2_D9_D10,
    PRESET_W2_A0_A1,

    // Width = 4 (MSB -> LSB)
    PRESET_W4_D4_D7,  // D4, D5, D6, D7 (PORTD 4~7)
    PRESET_W4_A0_A3,  // A0, A1, A2, A3 (PORTC 0~3)
    PRESET_W4_D9_D12, // D9, D10, D11, D12 (PORTB 1~4)

    // Width = 8 (MSB -> LSB)
    PRESET_W8_A0_D7   // A0~A3 (Bit 7~4) + D4~D7 (Bit 3~0)
};

class SWP2P {
public:
    SWP2P(uint8_t nodeId);
    void begin(bool clkIsOutput, DataPreset preset, unsigned long clkFreq = 100000UL);
    bool send(uint8_t destId, uint8_t data);
    bool available();
    uint8_t read();
    bool isSending() const { return GET_FLAG(FLAG_IS_SENDING); }
    bool isBusy() const { return GET_FLAG(FLAG_IS_BUSY); }

    static void _onClkEdge();
    static void _onBusyEdge();
    static void _onAckEdge();

private:
    enum TxState : uint8_t { TX_IDLE, TX_PENDING, TX_ARB, TX_DATA, TX_RELEASE, TX_DONE, TX_LOST };
    enum RxState : uint8_t { RX_IDLE, RX_ADDR, RX_SRC, RX_DATA, RX_ACK };

    static uint8_t _nodeId;
    static bool _clkIsOutput;
    static DataPreset _preset;
    static uint8_t _dataWidth;
    static unsigned long _clkFreq;

    // Rx FIFO
    static volatile uint8_t _rxFifo[SWP2P_FIFO_DEPTH];
    static volatile uint8_t _rxHead;
    static volatile uint8_t _rxTail;
    static volatile uint8_t _rxCount;

    // Tx State
    static volatile TxState _txState;
    static uint8_t _txDestId;
    static uint8_t _txData;
    static volatile uint16_t _txArbReg;
    static volatile uint8_t _txDataReg;
    static volatile uint8_t _arbChunkCount;
    static volatile uint8_t _txDataChunkCount;
    static volatile uint8_t _arbMyChunk;

    // Rx State
    static volatile RxState _rxState;
    static volatile uint8_t _rxAddrByte;
    static volatile uint8_t _rxSrcByte;
    static volatile uint8_t _rxDataByte;
    static volatile uint8_t _rxChunkCount;

    static inline void _driveDataChunk(uint8_t chunkVal) __attribute__((always_inline));
    static inline uint8_t _readDataChunk() __attribute__((always_inline));
    static inline void _dataRelease() __attribute__((always_inline));

    // Pure Open-Drain Control
    static inline void _busyDriveLow() { DDRD |= (1 << DDD3); }
    static inline void _busyRelease()  { DDRD &= ~(1 << DDD3); }
    static inline bool _busyRead()      { return (PIND & (1 << PIND3)) != 0; }
    static inline void _ackDriveLow()   { DDRB |= (1 << DDB0); }
    static inline void _ackRelease()    { DDRB &= ~(1 << DDB0); }
    static inline bool _ackRead()       { return (PINB & (1 << PINB0)) != 0; }

    static void setupTimer1(unsigned long freq);
    static void stopTimer1();
    static void _fifoPush(uint8_t val);
    static uint8_t _arbCycles();

    static uint8_t _arbCyclesCached;
    static uint8_t _dataMaskCached;
    static uint8_t _arbShiftAmt;
    static uint8_t _txShiftAmt;
};

#endif
