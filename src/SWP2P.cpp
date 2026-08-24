#include "SWP2P.h"

static_assert((SWP2P_FIFO_DEPTH & (SWP2P_FIFO_DEPTH - 1)) == 0, "SWP2P_FIFO_DEPTH must be a power of 2");

uint8_t SWP2PBase::_nodeId = 0;
volatile uint8_t SWP2PBase::_rxFifo[SWP2P_FIFO_DEPTH];
volatile uint8_t SWP2PBase::_rxHead = 0;
volatile uint8_t SWP2PBase::_rxTail = 0;
volatile uint8_t SWP2PBase::_rxCount = 0;

volatile uint8_t SWP2PBase::_txState = 0;
uint8_t SWP2PBase::_txDestId = 0;
uint8_t SWP2PBase::_txData = 0;
volatile uint16_t SWP2PBase::_txArbReg = 0;
volatile uint8_t SWP2PBase::_txDataReg = 0;
volatile uint8_t SWP2PBase::_arbChunkCount = 0;
volatile uint8_t SWP2PBase::_txDataChunkCount = 0;
volatile uint8_t SWP2PBase::_arbMyChunk = 0;

volatile uint8_t SWP2PBase::_rxState = 0;
volatile uint8_t SWP2PBase::_rxAddrByte = 0;
volatile uint8_t SWP2PBase::_rxSrcByte = 0;
volatile uint8_t SWP2PBase::_rxDataByte = 0;
volatile uint8_t SWP2PBase::_rxChunkCount = 0;

void (*SWP2PBase::_isrClkCallback)() = nullptr;
void (*SWP2PBase::_isrBusyCallback)() = nullptr;

void SWP2PBase::_fifoPush(uint8_t val) {
    if (_rxCount >= SWP2P_FIFO_DEPTH) return;
    _rxFifo[_rxHead] = val;
    _rxHead = (_rxHead + 1) & (SWP2P_FIFO_DEPTH - 1);
    _rxCount++;
}

void SWP2PBase::setupTimer1(unsigned long freq) {
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    uint16_t ocrValue = (uint16_t)((F_CPU / (2UL * freq)) - 1);
    OCR1A = ocrValue;
    TCCR1A |= (1 << COM1A0);
    TCCR1B |= (1 << WGM12) | (1 << CS10);
}

void SWP2PBase::stopTimer1() { TCCR1B = 0; }

ISR(INT0_vect) {
    if (SWP2PBase::_isrClkCallback) SWP2PBase::_isrClkCallback();
}

ISR(INT1_vect) {
    if (SWP2PBase::_isrBusyCallback) SWP2PBase::_isrBusyCallback();
}

ISR(PCINT0_vect) {
    // Ack edge handler
}
