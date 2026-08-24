#include "SWP2P.h"

static_assert((SWP2P_FIFO_DEPTH & (SWP2P_FIFO_DEPTH - 1)) == 0, "SWP2P_FIFO_DEPTH must be a power of 2");

static uint8_t g_nodeId = 0;
static bool g_clkIsOutput = false;
static unsigned long g_clkFreq = 40000UL;

static volatile bool g_isSending = false;
static volatile bool g_isBusy = false;

static volatile uint8_t g_rxFifo[SWP2P_FIFO_DEPTH];
static volatile uint8_t g_rxHead = 0;
static volatile uint8_t g_rxTail = 0;
static volatile uint8_t g_rxCount = 0;

static volatile SWP2P::TxState g_txState = SWP2P::TX_IDLE;
static uint8_t g_txDestId = 0;
static uint8_t g_txData = 0;
static int8_t g_arbChunkIdx = -1;
static uint8_t g_arbMyChunk = 0;
static int8_t g_txDataChunkIdx = -1;

static volatile SWP2P::RxState g_rxState = SWP2P::RX_IDLE;
static uint8_t g_rxAddrByte = 0;
static uint8_t g_rxDataByte = 0;
static int8_t g_rxAddrChunkIdx = -1;
static int8_t g_rxDataChunkIdx = -1;

static bool g_isMyPacket = false;
static bool g_rxCapturedThisFrame = false;

// Fast Direct Control Macros
#define BUSY_DRIVE_LOW() { DDRD |= (1 << DDD3); PORTD &= ~(1 << PORTD3); }
#define BUSY_RELEASE()   { DDRD &= ~(1 << DDD3); PORTD &= ~(1 << PORTD3); }
#define BUSY_READ()      ((PIND & (1 << PIND3)) != 0)

#define ACK_DRIVE_LOW()  { DDRB |= (1 << DDB0); PORTB &= ~(1 << PORTB0); }
#define ACK_RELEASE()    { DDRB &= ~(1 << DDB0); PORTB &= ~(1 << PORTB0); }
#define ACK_READ()       ((PINB & (1 << PINB0)) != 0)

#define DATA_RELEASE()   { DDRD &= 0x0F; PORTD &= 0x0F; }

// Data Pins: D4~D7
static inline void DRIVE_DATA_CHUNK(uint8_t v) {
    DDRD |= 0xF0;
    PORTD = (PORTD & 0x0F) | ((v & 0x0F) << 4);
}

static inline uint8_t READ_DATA_CHUNK() {
    return (PIND >> 4) & 0x0F;
}

static inline void FIFO_PUSH(uint8_t val) {
    if (g_rxCount >= SWP2P_FIFO_DEPTH) return;
    g_rxFifo[g_rxHead] = val;
    g_rxHead = (g_rxHead + 1) & (SWP2P_FIFO_DEPTH - 1);
    g_rxCount++;
}

SWP2P::SWP2P(uint8_t nodeId) {
    g_nodeId = nodeId;
}

void SWP2P::begin(bool clkIsOutput, uint8_t* dataPins, uint8_t dataWidth, unsigned long clkFreq) {
    g_clkIsOutput = clkIsOutput;
    g_clkFreq = clkFreq;

    DDRD &= ~(1 << DDD2);

    BUSY_RELEASE();
    ACK_RELEASE();
    DATA_RELEASE();

    if (g_clkIsOutput) {
        DDRB |= (1 << DDB1);
        setupTimer1(g_clkFreq);
    }

    // INT0 (CLK Any Logical Change)
    EICRA = (EICRA & ~((1 << ISC01) | (1 << ISC00))) | (1 << ISC00);
    EIMSK |= (1 << INT0);

    // INT1 (BUSY Any Logical Change)
    EICRA = (EICRA & ~((1 << ISC11) | (1 << ISC10))) | (1 << ISC10);
    EIMSK |= (1 << INT1);

    // PCINT0 (ACK)
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0);

    sei();
}

void SWP2P::setupTimer1(unsigned long freq) {
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    uint16_t ocrValue = (uint16_t)((F_CPU / (2UL * freq)) - 1);
    OCR1A = ocrValue;
    TCCR1A |= (1 << COM1A0);
    TCCR1B |= (1 << WGM12) | (1 << CS10);
}

void SWP2P::stopTimer1() {
    TCCR1B = 0;
}

bool SWP2P::send(uint8_t destId, uint8_t data) {
    if (g_isSending || g_isBusy) return false;
    g_txDestId = destId;
    g_txData = data;
    g_txState = TX_PENDING;
    g_isSending = true;
    return true;
}

bool SWP2P::available() {
    return g_rxCount > 0;
}

uint8_t SWP2P::read() {
    if (g_rxCount == 0) return 0;
    uint8_t val = g_rxFifo[g_rxTail];
    g_rxTail = (g_rxTail + 1) & (SWP2P_FIFO_DEPTH - 1);

    uint8_t sreg = SREG;
    cli();
    g_rxCount--;
    SREG = sreg;

    return val;
}

bool SWP2P::isSending() const { return g_isSending; }
bool SWP2P::isBusy() const { return g_isBusy; }

ISR(INT0_vect) {
    bool clkHigh = (PIND & (1 << PIND2)) != 0;

    if (clkHigh) {
        if (g_txState == SWP2P::TX_PENDING) {
            BUSY_DRIVE_LOW();
            g_arbChunkIdx = 1;
            g_txState = SWP2P::TX_ARB;

            g_arbMyChunk = (g_txDestId >> 4) & 0x0F;
            DRIVE_DATA_CHUNK(g_arbMyChunk);
            return;
        }

        switch (g_txState) {
            case SWP2P::TX_ARB: {
                if (g_arbChunkIdx < 0) {
                    g_txDataChunkIdx = 1;
                    DRIVE_DATA_CHUNK((g_txData >> 4) & 0x0F);
                    g_txDataChunkIdx--;
                    g_txState = SWP2P::TX_DATA;
                    break;
                }
                g_arbMyChunk = (g_txDestId >> (g_arbChunkIdx * 4)) & 0x0F;
                DRIVE_DATA_CHUNK(g_arbMyChunk);
                break;
            }
            case SWP2P::TX_DATA: {
                if (g_txDataChunkIdx < 0) {
                    g_txState = SWP2P::TX_RELEASE;
                    break;
                }
                DRIVE_DATA_CHUNK(g_txData & 0x0F);
                g_txDataChunkIdx--;
                break;
            }
            case SWP2P::TX_RELEASE:
                DATA_RELEASE();
                g_txState = SWP2P::TX_DONE;
                break;
            case SWP2P::TX_DONE:
                BUSY_RELEASE();
                g_isSending = false;
                g_txState = SWP2P::TX_IDLE;
                break;
            default:
                break;
        }
    } else {
        if (g_txState == SWP2P::TX_ARB) {
            asm volatile("nop\n\tnop\n\tnop\n\tnop\n\t");
            uint8_t busVal = READ_DATA_CHUNK();
            if ((g_arbMyChunk & ~busVal) != 0) {
                DATA_RELEASE();
                BUSY_RELEASE();
                g_isSending = false;
                g_txState = SWP2P::TX_LOST;
            } else {
                g_arbChunkIdx--;
            }
            return;
        }

        if (g_txState != SWP2P::TX_IDLE) return;

        // Signal Settling Delay for High Speed RX
        //asm volatile("nop\n\tnop\n\tnop\n\tnop\n\t");
        // 4.7K 대신 1K를 써보고 있기 때문에 이건 주석처리.

        switch (g_rxState) {
            case SWP2P::RX_ADDR: {
                if (g_rxAddrChunkIdx < 0) {
                    g_rxAddrChunkIdx = 1;
                    g_rxAddrByte = 0;
                }
                uint8_t v = READ_DATA_CHUNK();
                g_rxAddrByte = (g_rxAddrByte << 4) | (v & 0x0F);
                g_rxAddrChunkIdx--;
                if (g_rxAddrChunkIdx < 0) {
                    g_isMyPacket = (g_rxAddrByte == g_nodeId || g_rxAddrByte == 0xFF);
                    g_rxDataChunkIdx = 1;
                    g_rxDataByte = 0;
                    g_rxState = SWP2P::RX_DATA;
                }
                break;
            }
            case SWP2P::RX_DATA: {
                uint8_t v = READ_DATA_CHUNK();
                g_rxDataByte = (g_rxDataByte << 4) | (v & 0x0F);
                g_rxDataChunkIdx--;
                if (g_rxDataChunkIdx < 0) {
                    if (g_isMyPacket && !g_rxCapturedThisFrame) {
                        FIFO_PUSH(g_rxDataByte);
                        g_rxCapturedThisFrame = true;
                        g_rxState = SWP2P::RX_ACK;
                    } else {
                        g_rxState = SWP2P::RX_IDLE;
                    }
                }
                break;
            }
            case SWP2P::RX_ACK:
                ACK_DRIVE_LOW();
                g_rxState = SWP2P::RX_IDLE;
                break;
            default:
                break;
        }
    }
}

ISR(INT1_vect) {
    bool idle = BUSY_READ();
    g_isBusy = !idle;

    if (idle) {
        ACK_RELEASE();
        g_rxCapturedThisFrame = false;
        g_rxState = SWP2P::RX_IDLE;
        g_rxAddrChunkIdx = -1;
        g_rxDataChunkIdx = -1;
        g_rxAddrByte = 0;

        if (g_txState == SWP2P::TX_LOST) {
            g_txState = SWP2P::TX_PENDING;
            g_isSending = true;
        }
    } else {
        if (g_txState == SWP2P::TX_IDLE) g_rxState = SWP2P::RX_ADDR;
    }
}

ISR(PCINT0_vect) {
    bool ackHigh = ACK_READ();
    if (ackHigh) return;
}
