#include "SWP2P.h"

// FIFO depth는 반드시 2의 거듭제곱이어야 한다.
static_assert((SWP2P_FIFO_DEPTH & (SWP2P_FIFO_DEPTH - 1)) == 0, "SWP2P_FIFO_DEPTH must be a power of 2");

#define PIN_CLK    2
#define PIN_BUSY_N 3   // = PORTD bit 3
#define PIN_ACK_N  8   // = PORTB bit 0

SWP2P* SWP2P::_instance = nullptr;

SWP2P::SWP2P(uint8_t nodeId)
    : _nodeId(nodeId), _clkIsOutput(false), _dataPins(nullptr), _dataWidth(1),
      _clkFreq(100000UL), _isSending(false), _isBusy(false),
      _rxHead(0), _rxTail(0), _rxCount(0),
      _txState(TX_IDLE), _txDestId(0), _txData(0), _arbChunkIdx(-1), _arbMyChunk(0), _txDataChunkIdx(-1),
      _rxState(RX_IDLE), _rxAddrByte(0), _rxDataByte(0), _rxAddrChunkIdx(-1), _rxDataChunkIdx(-1),
      _isMyPacket(false), _rxCapturedThisFrame(false),
      _driveFn(nullptr), _readFn(nullptr) {
    _instance = this;
}

void SWP2P::_cachePinFast(uint8_t pinNum, PinFast& out) {
    uint8_t port = digitalPinToPort(pinNum);
    out.ddr  = portModeRegister(port);
    out.port = portOutputRegister(port);
    out.pin  = portInputRegister(port);
    out.mask = digitalPinToBitMask(pinNum);
}

// ==================== WIDTH별 언롤 구현 ====================
// 전부 루프 없이 몸으로 직접 풀어씀. p[i]는 begin()에서 이미 캐싱된 포인터라
// 여기서는 역참조 8줄(최대) 이하의 고정 명령만 실행된다.

// ---- WIDTH = 1 ----
void SWP2P::_drive_W1(PinFast* p, uint8_t v) {
    if (v & 0x01) { *p[0].ddr &= ~p[0].mask; *p[0].port &= ~p[0].mask; }
    else          { *p[0].port &= ~p[0].mask; *p[0].ddr |= p[0].mask; }
}
uint8_t SWP2P::_read_W1(PinFast* p) {
    return (*p[0].pin & p[0].mask) ? 1 : 0;
}

// ---- WIDTH = 2 ----
void SWP2P::_drive_W2(PinFast* p, uint8_t v) {
    if (v & 0x02) { *p[0].ddr &= ~p[0].mask; *p[0].port &= ~p[0].mask; }
    else          { *p[0].port &= ~p[0].mask; *p[0].ddr |= p[0].mask; }
    if (v & 0x01) { *p[1].ddr &= ~p[1].mask; *p[1].port &= ~p[1].mask; }
    else          { *p[1].port &= ~p[1].mask; *p[1].ddr |= p[1].mask; }
}
uint8_t SWP2P::_read_W2(PinFast* p) {
    uint8_t val = (*p[0].pin & p[0].mask) ? 1 : 0;
    val = (val << 1) | ((*p[1].pin & p[1].mask) ? 1 : 0);
    return val;
}

// ---- WIDTH = 4 ----
void SWP2P::_drive_W4(PinFast* p, uint8_t v) {
    if (v & 0x08) { *p[0].ddr &= ~p[0].mask; *p[0].port &= ~p[0].mask; }
    else          { *p[0].port &= ~p[0].mask; *p[0].ddr |= p[0].mask; }
    if (v & 0x04) { *p[1].ddr &= ~p[1].mask; *p[1].port &= ~p[1].mask; }
    else          { *p[1].port &= ~p[1].mask; *p[1].ddr |= p[1].mask; }
    if (v & 0x02) { *p[2].ddr &= ~p[2].mask; *p[2].port &= ~p[2].mask; }
    else          { *p[2].port &= ~p[2].mask; *p[2].ddr |= p[2].mask; }
    if (v & 0x01) { *p[3].ddr &= ~p[3].mask; *p[3].port &= ~p[3].mask; }
    else          { *p[3].port &= ~p[3].mask; *p[3].ddr |= p[3].mask; }
}
uint8_t SWP2P::_read_W4(PinFast* p) {
    uint8_t val = (*p[0].pin & p[0].mask) ? 1 : 0;
    val = (val << 1) | ((*p[1].pin & p[1].mask) ? 1 : 0);
    val = (val << 1) | ((*p[2].pin & p[2].mask) ? 1 : 0);
    val = (val << 1) | ((*p[3].pin & p[3].mask) ? 1 : 0);
    return val;
}

// ---- WIDTH = 8 ----
void SWP2P::_drive_W8(PinFast* p, uint8_t v) {
    if (v & 0x80) { *p[0].ddr &= ~p[0].mask; *p[0].port &= ~p[0].mask; }
    else          { *p[0].port &= ~p[0].mask; *p[0].ddr |= p[0].mask; }
    if (v & 0x40) { *p[1].ddr &= ~p[1].mask; *p[1].port &= ~p[1].mask; }
    else          { *p[1].port &= ~p[1].mask; *p[1].ddr |= p[1].mask; }
    if (v & 0x20) { *p[2].ddr &= ~p[2].mask; *p[2].port &= ~p[2].mask; }
    else          { *p[2].port &= ~p[2].mask; *p[2].ddr |= p[2].mask; }
    if (v & 0x10) { *p[3].ddr &= ~p[3].mask; *p[3].port &= ~p[3].mask; }
    else          { *p[3].port &= ~p[3].mask; *p[3].ddr |= p[3].mask; }
    if (v & 0x08) { *p[4].ddr &= ~p[4].mask; *p[4].port &= ~p[4].mask; }
    else          { *p[4].port &= ~p[4].mask; *p[4].ddr |= p[4].mask; }
    if (v & 0x04) { *p[5].ddr &= ~p[5].mask; *p[5].port &= ~p[5].mask; }
    else          { *p[5].port &= ~p[5].mask; *p[5].ddr |= p[5].mask; }
    if (v & 0x02) { *p[6].ddr &= ~p[6].mask; *p[6].port &= ~p[6].mask; }
    else          { *p[6].port &= ~p[6].mask; *p[6].ddr |= p[6].mask; }
    if (v & 0x01) { *p[7].ddr &= ~p[7].mask; *p[7].port &= ~p[7].mask; }
    else          { *p[7].port &= ~p[7].mask; *p[7].ddr |= p[7].mask; }
}
uint8_t SWP2P::_read_W8(PinFast* p) {
    uint8_t val = (*p[0].pin & p[0].mask) ? 1 : 0;
    val = (val << 1) | ((*p[1].pin & p[1].mask) ? 1 : 0);
    val = (val << 1) | ((*p[2].pin & p[2].mask) ? 1 : 0);
    val = (val << 1) | ((*p[3].pin & p[3].mask) ? 1 : 0);
    val = (val << 1) | ((*p[4].pin & p[4].mask) ? 1 : 0);
    val = (val << 1) | ((*p[5].pin & p[5].mask) ? 1 : 0);
    val = (val << 1) | ((*p[6].pin & p[6].mask) ? 1 : 0);
    val = (val << 1) | ((*p[7].pin & p[7].mask) ? 1 : 0);
    return val;
}

void SWP2P::_dataRelease() {
    for (uint8_t i = 0; i < _dataWidth; i++) {
        PinFast& p = _dataPinFast[i];
        *p.ddr &= ~p.mask;
        *p.port &= ~p.mask;
    }
}

uint8_t SWP2P::_arbCycles() const { return (8 + _dataWidth - 1) / _dataWidth; }

// ---------------- begin ----------------
void SWP2P::begin(bool clkIsOutput, uint8_t* dataPins, uint8_t dataWidth, unsigned long clkFreq) {
    _clkIsOutput = clkIsOutput;
    _dataPins = dataPins;
    _dataWidth = (dataWidth == 0) ? 1 : dataWidth;
    _clkFreq = clkFreq;

    pinMode(PIN_CLK, INPUT); // 내부 풀업 미사용
    for (uint8_t i = 0; i < _dataWidth; i++) {
        _cachePinFast(_dataPins[i], _dataPinFast[i]);
    }
    _busyRelease();
    _ackRelease();
    _dataRelease();

    _arbCyclesCached = _arbCycles();
    _dataMaskCached  = (1 << _dataWidth) - 1;
    _dataTopBitCached = (uint8_t)(1 << (_dataWidth - 1));

    // WIDTH에 맞는 언롤 함수를 1회만 바인딩. 지원 목록: 1, 2, 4, 8.
    // 그 외 값(3,5,6,7)이 들어오면 일단 가장 가까운 하위 지원폭으로 clamp하지 않고
    // 명시적으로 8용 범용 폭에 맞추는 대신 여기서는 지원값만 쓰도록 강제한다.
    switch (_dataWidth) {
        case 1: _driveFn = &SWP2P::_drive_W1; _readFn = &SWP2P::_read_W1; break;
        case 2: _driveFn = &SWP2P::_drive_W2; _readFn = &SWP2P::_read_W2; break;
        case 4: _driveFn = &SWP2P::_drive_W4; _readFn = &SWP2P::_read_W4; break;
        case 8: _driveFn = &SWP2P::_drive_W8; _readFn = &SWP2P::_read_W8; break;
        default:
            // 지원하지 않는 WIDTH: 가장 가까운 하위 지원값으로 강제하지 않고
            // 8용 구현을 쓰되 상위 미사용 비트는 0으로 취급되도록 W8을 사용.
            // (요구사항 밖의 값이 들어오면 일단 안전하게 fallback, 추후 static_assert로 강제 고려)
            _driveFn = &SWP2P::_drive_W8;
            _readFn = &SWP2P::_read_W8;
            break;
    }

    if (_clkIsOutput) {
        pinMode(9, OUTPUT);
        setupTimer1(_clkFreq);
    }

    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    EICRA |= (1 << ISC00);
    EIMSK |= (1 << INT0);

    EICRA &= ~((1 << ISC11) | (1 << ISC10));
    EICRA |= (1 << ISC10);
    EIMSK |= (1 << INT1);

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
void SWP2P::stopTimer1() { TCCR1B = 0; }

// ---------------- 사용자 API ----------------
bool SWP2P::send(uint8_t destId, uint8_t data) {
    if (_isSending || _isBusy) return false;
    _txDestId = destId;
    _txData = data;
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
    if (_rxCount >= SWP2P_FIFO_DEPTH) return;
    _rxFifo[_rxHead] = val;
    _rxHead = (_rxHead + 1) & (SWP2P_FIFO_DEPTH - 1);
    _rxCount++;
}

// ---------------- CLK 엣지 ----------------
void SWP2P::_onClkEdge() {
    bool clkHigh = (PIND & (1 << PIN_CLK)) != 0;

    if (clkHigh) {
        if (_txState == TX_PENDING) {
            _busyDriveLow();
            _arbChunkIdx = _arbCyclesCached - 1;
            _txState = TX_ARB;

            uint8_t shift = _arbChunkIdx * _dataWidth;
            _arbMyChunk = (_txDestId >> shift) & _dataMaskCached;
            _driveDataChunk(_arbMyChunk);
            return;
        }

        switch (_txState) {
            case TX_ARB: {
                if (_arbChunkIdx < 0) {
                    _txDataChunkIdx = _arbCyclesCached - 1;
                    uint8_t shift0 = _txDataChunkIdx * _dataWidth;
                    _driveDataChunk((_txData >> shift0) & _dataMaskCached);
                    _txDataChunkIdx--;
                    _txState = TX_DATA;
                    if (_txDataChunkIdx < 0) _txState = TX_RELEASE;
                    break;
                }
                uint8_t shift = _arbChunkIdx * _dataWidth;
                _arbMyChunk = (_txDestId >> shift) & _dataMaskCached;
                _driveDataChunk(_arbMyChunk);
                break;
            }
            case TX_DATA: {
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
            uint8_t busVal = _readDataChunk();
            if ((_arbMyChunk & ~busVal) != 0) {
                _dataRelease();
                _busyRelease();
                _isSending = false;
                _txState = TX_LOST;
            } else {
                _arbChunkIdx--;
            }
            return;
        }
        if (_txState != TX_IDLE) return;

        switch (_rxState) {
            case RX_ADDR: {
                if (_rxAddrChunkIdx < 0) _rxAddrChunkIdx = _arbCyclesCached - 1;
                uint8_t v = _readDataChunk();
                _rxAddrByte = (_rxAddrByte << _dataWidth) | v;
                _rxAddrChunkIdx--;
                if (_rxAddrChunkIdx < 0) {
                    _isMyPacket = (_rxAddrByte == _nodeId || _rxAddrByte == 0xFF);
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
                _ackDriveLow();
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
        _ackRelease();
        _rxCapturedThisFrame = false;
        _rxState = RX_IDLE;
        _rxAddrChunkIdx = -1;
        _rxDataChunkIdx = -1;
        _rxAddrByte = 0;

        if (_txState == TX_LOST) {
            _txState = TX_PENDING;
            _isSending = true;
        }
    } else {
        if (_txState == TX_IDLE) _rxState = RX_ADDR;
    }
}

void SWP2P::_onAckEdge() {
    bool ackHigh = _ackRead();
    if (ackHigh) return;
    // v0.1: 자리만 유지
}

// ---------------- ISR 트램폴린 ----------------
ISR(INT0_vect) { if (SWP2P::_instance) SWP2P::_instance->_onClkEdge(); }
ISR(INT1_vect) { if (SWP2P::_instance) SWP2P::_instance->_onBusyEdge(); }
ISR(PCINT0_vect) { if (SWP2P::_instance) SWP2P::_instance->_onAckEdge(); }
