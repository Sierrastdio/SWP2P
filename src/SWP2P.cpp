#include "SWP2P.h"
static_assert((SWP2P_FIFO_DEPTH & (SWP2P_FIFO_DEPTH - 1)) == 0, "SWP2P_FIFO_DEPTH must be a power of 2");

uint8_t SWP2PBase::_nodeId = 0;
volatile uint8_t SWP2PBase::_rxFifo[SWP2P_FIFO_DEPTH];
volatile uint8_t SWP2PBase::_rxHead = 0;
volatile uint8_t SWP2PBase::_rxTail = 0;
volatile uint8_t SWP2PBase::_rxCount = 0;

// _txState -> GPIOR1, _rxState -> GPIOR2 (SWP2P.h의 #define 참고, 별도 정의 불필요)
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

volatile uint8_t SWP2PBase::_rxAddrByte = 0;
volatile uint8_t SWP2PBase::_rxSrcByte = 0;
volatile uint8_t SWP2PBase::_rxDataByte = 0;
volatile uint8_t SWP2PBase::_rxChunkCount = 0;
volatile uint8_t SWP2PBase::_rxLen = 0;
volatile uint8_t SWP2PBase::_rxByteIdx = 0;

void SWP2PBase::_fifoPush(uint8_t val) {
    if (_rxCount >= SWP2P_FIFO_DEPTH) return;
    _rxFifo[_rxHead] = val;
    // _rxHead = (_rxHead + 1) % SWP2P_FIFO_DEPTH 연산을 비트 마스크로 고속 처리
    // SWP2P_FIFO_DEPTH가 16(0x10 = 0001 0000)일 때, (SWP2P_FIFO_DEPTH - 1) = 15(0x0F = 0000 1111)
    // 0000 1111 마스킹을 통해 인덱스가 15를 초과하면 자동으로 0으로 순환(Circular Queue)
    _rxHead = (_rxHead + 1) & (SWP2P_FIFO_DEPTH - 1);
    _rxCount++;
}

void SWP2PBase::setupTimer1(unsigned long freq) {
    TCCR1A = 0; // 0x00 (0000 0000) : Timer1 제어 레지스터 A 초기화
    TCCR1B = 0; // 0x00 (0000 0000) : Timer1 제어 레지스터 B 초기화
    TCNT1  = 0; // 0x0000 : 타이머 카운터 값 0으로 초기화

    // CTC 모드 토글 주파수 공식: F_oc1a = F_CPU / (2 * N * (1 + OCR1A))  (단, 분주비 N = 1)
    // 설정 주파수(freq)에 맞는 OCR1A 비교 일치 값 계산
    uint16_t ocrValue = (uint16_t)((F_CPU / (2UL * freq)) - 1);
    OCR1A = ocrValue;

    // COM1A0 = 1 (비트 6 set) -> CTC 비교 일치 발생 시 OC1A(PB1 / Arduino D9) 핀 토글(Toggle)
    // TCCR1A |= (1 << COM1A0) -> 0x40 (0100 0000)
    TCCR1A |= (1 << COM1A0);

    // WGM12 = 1 (비트 3 set) -> CTC 모드 설정 (TCNT1이 OCR1A에 도달하면 0으로 리셋)
    // CS10  = 1 (비트 0 set) -> 분주비 1 (No Prescaling, 클럭 직결)
    // TCCR1B |= (1 << WGM12) | (1 << CS10) -> (1 << 3) | (1 << 0) = 0x08 | 0x01 = 0x09 (0000 1001)
    TCCR1B |= (1 << WGM12) | (1 << CS10);
}

void SWP2PBase::stopTimer1() {
    // CS12, CS11, CS10 비트를 모두 0으로 만들어 클럭 공급 차단 (타이머 정지)
    TCCR1B = 0; // 0x00 (0000 0000)
}

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
