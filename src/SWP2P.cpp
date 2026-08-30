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
uint8_t SWP2PBase::_txBuffer[SWP2P_MAX_BURST];
uint8_t SWP2PBase::_txLen = 0;
uint8_t SWP2PBase::_txIdx = 0;
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
volatile uint8_t SWP2PBase::_rxLen = 0;
volatile uint8_t SWP2PBase::_rxByteIdx = 0;

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

// ---- ISR 정의는 더 이상 여기 없음 ----
// 함수 포인터 간접호출 오버헤드를 없애기 위해, 실제 사용하는 PRESET으로
// ISR을 컴파일타임에 직접 바인딩해야 한다. 노드를 선언한 .ino/.cpp에서
// 아래처럼 한 번만 호출하면 된다:
//
//   SWP2P<PRESET_W1_D4> node(1);
//   SWP2P_BIND_ISRS(PRESET_W1_D4);
//
// 또한 기존 ISR(PCINT0_vect)(ACK_N 감지용, 본체는 비어있었음)는 제거되고
// Input Capture Unit(TIMER1_CAPT_vect) 기반으로 대체되었다 (SWP2P.h 참고).
