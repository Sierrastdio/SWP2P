#include "SWP2P.h"

// FIFO depth는 반드시 2의 거듭제곱이어야 한다.
// _fifoPush/read()에서 %가 아닌 비트마스크(& (DEPTH-1))로 인덱스를 감싸기 때문.
// (AVR엔 나눗셈 명령이 없어 %는 소프트웨어 루프로 처리되어 ISR 안에서는 특히 비싸다.
//  2의 거듭제곱이면 컴파일러가 %도 자동으로 마스크로 바꿔주지만, 명시적으로 고정해두면
//  향후 SWP2P_FIFO_DEPTH를 실수로 4→5 같은 값으로 바꿔도 조용히 깨지지 않고 여기서 걸린다.)
static_assert((SWP2P_FIFO_DEPTH & (SWP2P_FIFO_DEPTH - 1)) == 0, "SWP2P_FIFO_DEPTH must be a power of 2");

// 고정 핀 (하드웨어 제약: INT0=D2, INT1=D3, PCINT0 그룹의 D8)
#define PIN_CLK    2
#define PIN_BUSY_N 3
#define PIN_ACK_N  8

SWP2P* SWP2P::_instance = nullptr;

SWP2P::SWP2P(uint8_t nodeId)
    : _nodeId(nodeId), _clkIsOutput(false), _dataPins(nullptr), _dataWidth(1),
      _clkFreq(100000UL), _isSending(false), _isBusy(false),
      _rxHead(0), _rxTail(0), _rxCount(0),
      _txState(TX_IDLE), _txDestId(0), _txData(0), _arbChunkIdx(-1), _arbMyChunk(0), _txDataChunkIdx(-1),
      _rxState(RX_IDLE), _rxAddrByte(0), _rxDataByte(0), _rxAddrChunkIdx(-1), _rxDataChunkIdx(-1),
      _isMyPacket(false), _rxCapturedThisFrame(false) {
    _instance = this;
}

// ---------------- 핀 -> 레지스터 캐싱 (begin()에서 1회만 실행, 이후 ISR은 포인터만 역참조) ----------------
void SWP2P::_cachePinFast(uint8_t pinNum, PinFast& out) {
    uint8_t port = digitalPinToPort(pinNum);
    out.ddr  = portModeRegister(port);
    out.port = portOutputRegister(port);
    out.pin  = portInputRegister(port);
    out.mask = digitalPinToBitMask(pinNum);
}

// ---------------- open-drain 헬퍼 (전부 포트 레지스터 직접 접근, digitalWrite/pinMode 미사용) ----------------
void SWP2P::_busyDriveLow() { *_busyFast.port &= ~_busyFast.mask; *_busyFast.ddr |= _busyFast.mask; }
void SWP2P::_busyRelease()  { *_busyFast.ddr &= ~_busyFast.mask; *_busyFast.port |= _busyFast.mask; } // INPUT_PULLUP
bool SWP2P::_busyRead()     { return (*_busyFast.pin & _busyFast.mask) != 0; }

void SWP2P::_ackDriveLow() { *_ackFast.port &= ~_ackFast.mask; *_ackFast.ddr |= _ackFast.mask; }
void SWP2P::_ackRelease()  { *_ackFast.ddr &= ~_ackFast.mask; *_ackFast.port |= _ackFast.mask; }
bool SWP2P::_ackRead()     { return (*_ackFast.pin & _ackFast.mask) != 0; }

void SWP2P::_driveDataChunk(uint8_t chunkVal) {
    for (uint8_t i = 0; i < _dataWidth; i++) {
        bool bitVal = (chunkVal >> (_dataWidth - 1 - i)) & 0x01;
        PinFast& p = _dataPinFast[i];
        if (bitVal) { *p.ddr &= ~p.mask; *p.port |= p.mask; }   // 1 = release (INPUT_PULLUP)
        else        { *p.port &= ~p.mask; *p.ddr |= p.mask; }  // 0 = 구동 (OUTPUT LOW)
    }
}
uint8_t SWP2P::_readDataChunk() {
    uint8_t val = 0;
    for (uint8_t i = 0; i < _dataWidth; i++) {
        val <<= 1;
        val |= (*_dataPinFast[i].pin & _dataPinFast[i].mask) ? 1 : 0;
    }
    return val;
}
void SWP2P::_dataRelease() {
    for (uint8_t i = 0; i < _dataWidth; i++) {
        PinFast& p = _dataPinFast[i];
        *p.ddr &= ~p.mask;
        *p.port |= p.mask;
    }
}

uint8_t SWP2P::_arbCycles() const { return (8 + _dataWidth - 1) / _dataWidth; }

// ---------------- begin ----------------
void SWP2P::begin(bool clkIsOutput, uint8_t* dataPins, uint8_t dataWidth, unsigned long clkFreq) {
    _clkIsOutput = clkIsOutput;
    _dataPins = dataPins;
    _dataWidth = (dataWidth == 0) ? 1 : dataWidth;
    _clkFreq = clkFreq;

    pinMode(PIN_CLK, INPUT_PULLUP);
    _cachePinFast(PIN_BUSY_N, _busyFast);
    _cachePinFast(PIN_ACK_N, _ackFast);
    for (uint8_t i = 0; i < _dataWidth; i++) {
        _cachePinFast(_dataPins[i], _dataPinFast[i]);
    }
    _busyRelease();
    _ackRelease();
    _dataRelease();

    // ISR 핫패스 캐시: 나눗셈/시프트 결과를 1회만 계산해둔다 (매 클럭 엣지마다 재계산 방지)
    _arbCyclesCached = _arbCycles();
    _dataMaskCached  = (1 << _dataWidth) - 1;

    if (_clkIsOutput) {
        pinMode(9, OUTPUT); // Timer1 OC1A -> D2로 점퍼선 연결 필요
        setupTimer1(_clkFreq);
    }

    // INT0 (CLK) - 반드시 CHANGE: posedge=Tx 스텝, negedge=Rx 스텝 둘 다 필요
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    EICRA |= (1 << ISC00); // ISC01=0, ISC00=1 -> logical change
    EIMSK |= (1 << INT0);

    // INT1 (BUSY_N) - CHANGE
    EICRA &= ~((1 << ISC11) | (1 << ISC10));
    EICRA |= (1 << ISC10);
    EIMSK |= (1 << INT1);

    // PCINT0 (ACK_N, D8) - 하드웨어 특성상 CHANGE 고정, ISR 내부에서 레벨로 구분
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
    TCCR1A |= (1 << COM1A0);              // Toggle OC1A on Compare Match
    TCCR1B |= (1 << WGM12) | (1 << CS10); // CTC, no prescale
}
void SWP2P::stopTimer1() { TCCR1B = 0; }

// ---------------- 사용자 API ----------------
bool SWP2P::send(uint8_t destId, uint8_t data) {
    if (_isSending || _isBusy) return false; // 내부 가드
    _txDestId = destId;
    _txData = data;
    // 여기서 곧바로 busyDriveLow()를 호출하지 않는다.
    // main loop 컨텍스트에서 즉시 구동하면 CLK 위상과 무관한 임의 시점에
    // BUSY_N이 떨어지게 되어, 수신측이 "몇 번째 청크가 첫 청크인지" 착각하는
    // 레이스 컨디션이 생긴다 (관찰된 프레임 밀림 버그의 원인).
    // 대신 TX_PENDING만 세팅해두고, 실제 점유 시작은 다음 CLK posedge에서
    // _onClkEdge()가 첫 주소 청크 구동과 "동시에" 수행하도록 한다.
    _txState = TX_PENDING;
    _isSending = true;
    return true;
}

bool SWP2P::available() { return _rxCount > 0; }

uint8_t SWP2P::read() {
    if (_rxCount == 0) return 0;
    uint8_t val = _rxFifo[_rxTail];
    _rxTail = (_rxTail + 1) & (SWP2P_FIFO_DEPTH - 1);
    noInterrupts();
    _rxCount--;
    interrupts();
    return val;
}

void SWP2P::_fifoPush(uint8_t val) {
    if (_rxCount >= SWP2P_FIFO_DEPTH) return; // full -> drop
    _rxFifo[_rxHead] = val;
    _rxHead = (_rxHead + 1) & (SWP2P_FIFO_DEPTH - 1);
    _rxCount++;
}

// ---------------- CLK 엣지: 상승=Tx 스텝, 하강=Rx 스텝 ----------------
void SWP2P::_onClkEdge() {
    bool clkHigh = (PIND & (1 << PIN_CLK)) != 0; // PIN_CLK=D2 -> PORTD 고정 상수라 직접 접근

    if (clkHigh) {
        // TX_PENDING: send() 직후 첫 posedge.
        // BUSY_N 하강과 "중재 첫 청크(최상위 니블/비트)" 구동을 이 하나의 엣지 안에서
        // 동시에 처리한다. 이렇게 해야 슬레이브가 BUSY_N 하강 인터럽트를 받고 나서
        // 맞이하는 "바로 다음 negedge"가 항상 이 첫 청크와 정확히 일치하게 된다.
        if (_txState == TX_PENDING) {
            _busyDriveLow();                     // 물리적 버스 점유 시작 (이 posedge 시점으로 고정)
            _arbChunkIdx = _arbCyclesCached - 1;  // 중재 청크 인덱스 초기화 (최상위부터)
            _txState = TX_ARB;

            uint8_t shift = _arbChunkIdx * _dataWidth;
            _arbMyChunk = (_txDestId >> shift) & _dataMaskCached;
            _driveDataChunk(_arbMyChunk);         // 첫 주소 청크를 이 posedge에서 즉시 구동
            // 주의: 여기서 _arbChunkIdx를 감소시키지 않는다.
            // 원래 흐름상 감소는 "negedge에서 되읽기 검증 후"에만 일어난다(TX_ARB negedge 분기 참고).
            // 여기서 미리 감소시키면 negedge에서 한 번 더 감소되어 이중으로 줄어들고,
            // 그 결과 중재 청크를 1개 덜 구동한 채 데이터 단계로 조기 진입해버려
            // 슬레이브와 프레임 길이가 어긋나는 버그가 생긴다(실제 겪으신 "수신 침묵" 증상의 원인).
            return;                               // 이번 엣지는 여기서 종료 (TX_ARB 분기와 중복 실행 방지)
        }

        switch (_txState) {
            case TX_ARB: {
                if (_arbChunkIdx < 0) {
                    // 중재 마지막 클럭 다음 스텝: 이 클럭에 첫 데이터 청크를 즉시 구동
                    // (데이터 단계는 이미 유일한 송신자로 확정된 뒤라 되읽기 검증이 필요 없음)
                    _txDataChunkIdx = _arbCyclesCached - 1;
                    uint8_t shift0 = _txDataChunkIdx * _dataWidth;
                    _driveDataChunk((_txData >> shift0) & _dataMaskCached);
                    _txDataChunkIdx--;
                    _txState = TX_DATA;
                    if (_txDataChunkIdx < 0) _txState = TX_RELEASE;
                    break;
                }
                // 이 클럭에는 "구동만" 한다. 검증(되읽기)은 반클럭 뒤 negedge에서 -
                // 풀업이 라인을 실제로 HIGH까지 끌어올릴 시간(RC 정착시간)을 줘야 하기 때문.
                // 같은 엣지에서 바로 되읽으면 경쟁자가 없어도 매번 거짓 충돌로 오판한다.
                uint8_t shift = _arbChunkIdx * _dataWidth;
                _arbMyChunk = (_txDestId >> shift) & _dataMaskCached;
                _driveDataChunk(_arbMyChunk);
                break;
            }
            case TX_DATA: {
                // TX_ARB(혹은 TX_PENDING)에서 이미 첫 청크는 구동해뒀으므로, 여기서는 다음 청크부터 처리
                if (_txDataChunkIdx < 0) { _txState = TX_RELEASE; break; }
                uint8_t shift = _txDataChunkIdx * _dataWidth;
                uint8_t chunk = (_txData >> shift) & _dataMaskCached;
                _driveDataChunk(chunk);
                _txDataChunkIdx--;
                if (_txDataChunkIdx < 0) _txState = TX_RELEASE;
                break;
            }
            case TX_RELEASE:
                _dataRelease();
                _txState = TX_DONE;
                break;
            case TX_DONE:
                _busyRelease();
                _isSending = false;
                _txState = TX_IDLE;
                break;
            default:
                break;
        }
    } else {
        // Rx 스텝 (negedge)
        if (_txState == TX_ARB) {
            // 반클럭 지난 시점 - 풀업이 정착할 시간을 준 뒤 검증
            uint8_t busVal = _readDataChunk();
            if ((_arbMyChunk & ~busVal) != 0) {
                // 내가 1(release)로 보낸 자리에 버스가 0 -> 패배, 즉시 양보
                _dataRelease();
                _busyRelease();
                _isSending = false;
                _txState = TX_LOST;
            } else {
                _arbChunkIdx--;
            }
            return;
        }
        if (_txState != TX_IDLE) return; // 내가 pending/데이터/release/done 진행 중이면 수신 로직 skip

        switch (_rxState) {
            case RX_ADDR: {
                if (_rxAddrChunkIdx < 0) _rxAddrChunkIdx = _arbCyclesCached - 1; // 프레임 첫 진입
                uint8_t v = _readDataChunk();
                _rxAddrByte = (_rxAddrByte << _dataWidth) | v;
                _rxAddrChunkIdx--;
                if (_rxAddrChunkIdx < 0) {
                    _isMyPacket = (_rxAddrByte == _nodeId || _rxAddrByte == 0xFF); // 0xFF = 브로드캐스트
                    _rxDataChunkIdx = _arbCyclesCached - 1;
                    _rxDataByte = 0;
                    _rxState = RX_DATA;
                }
                break;
            }
            case RX_DATA: {
                uint8_t v = _readDataChunk();
                _rxDataByte = (_rxDataByte << _dataWidth) | v;
                _rxDataChunkIdx--;
                if (_rxDataChunkIdx < 0) {
                    if (_isMyPacket && !_rxCapturedThisFrame) {
                        _fifoPush(_rxDataByte);
                        _rxCapturedThisFrame = true;
                        _rxState = RX_ACK;
                    } else {
                        _rxState = RX_IDLE;
                    }
                }
                break;
            }
            case RX_ACK:
                _ackDriveLow(); // 1클럭 폭 ACK 시작 (HIGH 복귀는 _onBusyEdge의 idle 처리에서)
                _rxState = RX_IDLE;
                break;
            default:
                break;
        }
    }
}

void SWP2P::_onBusyEdge() {
    bool idle = _busyRead();
    _isBusy = !idle;

    if (idle) {
        _ackRelease(); // 프레임 종료 시 ACK 라인도 반드시 복귀
        _rxCapturedThisFrame = false;
        _rxState = RX_IDLE;
        _rxAddrChunkIdx = -1;
        _rxDataChunkIdx = -1;
        _rxAddrByte = 0;

        if (_txState == TX_LOST) {
            // 재시도도 동일하게 다음 posedge에서 동기화되도록 PENDING으로 복귀시킨다.
            // (여기서 곧바로 busyDriveLow()를 부르면 원래 버그가 재시도 경로에서 재발한다.)
            _txState = TX_PENDING;
            _isSending = true;
        }
    } else {
        if (_txState == TX_IDLE) _rxState = RX_ADDR;
    }
}

void SWP2P::_onAckEdge() {
    // PCINT는 CHANGE 전용이라 HIGH 복귀에도 걸림 - 레벨로 걸러낸다
    bool ackHigh = _ackRead();
    if (ackHigh) return; // 상승엣지(ACK 종료) - 무시

    // 하강엣지(ACK 시작) - 송신자 입장에서 "상대가 확인했다"는 신호
    // v0.1: 송신 완료 자체는 이미 TX_DONE에서 처리되므로 여기서는 자리만 유지
    // (재전송/타임아웃 로직 도입 시 이 지점에서 성공 플래그를 세운다)
}

// ---------------- ISR 트램폴린 ----------------
ISR(INT0_vect) { if (SWP2P::_instance) SWP2P::_instance->_onClkEdge(); }
ISR(INT1_vect) { if (SWP2P::_instance) SWP2P::_instance->_onBusyEdge(); }
ISR(PCINT0_vect) { if (SWP2P::_instance) SWP2P::_instance->_onAckEdge(); }
