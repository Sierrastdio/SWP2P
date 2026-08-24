#include "SWP2P.h"

static_assert((SWP2P_FIFO_DEPTH & (SWP2P_FIFO_DEPTH - 1)) == 0, "SWP2P_FIFO_DEPTH must be a power of 2");

// Static Members Initialization
uint8_t SWP2P::_nodeId = 0;
bool SWP2P::_clkIsOutput = false;
DataPreset SWP2P::_preset = PRESET_W4_D4_D7;
uint8_t SWP2P::_dataWidth = 4;
unsigned long SWP2P::_clkFreq = 100000UL;

volatile uint8_t SWP2P::_rxFifo[SWP2P_FIFO_DEPTH];
volatile uint8_t SWP2P::_rxHead = 0;
volatile uint8_t SWP2P::_rxTail = 0;
volatile uint8_t SWP2P::_rxCount = 0;

volatile SWP2P::TxState SWP2P::_txState = TX_IDLE;
uint8_t SWP2P::_txDestId = 0;
uint8_t SWP2P::_txData = 0;
volatile uint16_t SWP2P::_txArbReg = 0;
volatile uint8_t SWP2P::_txDataReg = 0;
volatile uint8_t SWP2P::_arbChunkCount = 0;
volatile uint8_t SWP2P::_txDataChunkCount = 0;
volatile uint8_t SWP2P::_arbMyChunk = 0;

volatile SWP2P::RxState SWP2P::_rxState = RX_IDLE;
volatile uint8_t SWP2P::_rxAddrByte = 0;
volatile uint8_t SWP2P::_rxSrcByte = 0;
volatile uint8_t SWP2P::_rxDataByte = 0;
volatile uint8_t SWP2P::_rxChunkCount = 0;

uint8_t SWP2P::_arbCyclesCached = 0;
uint8_t SWP2P::_dataMaskCached = 0;
uint8_t SWP2P::_arbShiftAmt = 0;
uint8_t SWP2P::_txShiftAmt = 0;

SWP2P::SWP2P(uint8_t nodeId) {
    _nodeId = nodeId;
}

// Ultra Fast Port Driving (No Loops, Direct Register Operations)
inline void SWP2P::_driveDataChunk(uint8_t chunkVal) {
    switch (_preset) {
        // Width = 1
        case PRESET_W1_D4:  if (!(chunkVal & 0x01)) DDRD |= (1 << DDD4); else DDRD &= ~(1 << DDD4); break;
        case PRESET_W1_D5:  if (!(chunkVal & 0x01)) DDRD |= (1 << DDD5); else DDRD &= ~(1 << DDD5); break;
        case PRESET_W1_D6:  if (!(chunkVal & 0x01)) DDRD |= (1 << DDD6); else DDRD &= ~(1 << DDD6); break;
        case PRESET_W1_D7:  if (!(chunkVal & 0x01)) DDRD |= (1 << DDD7); else DDRD &= ~(1 << DDD7); break;
        case PRESET_W1_D9:  if (!(chunkVal & 0x01)) DDRB |= (1 << DDB1); else DDRB &= ~(1 << DDB1); break;
        case PRESET_W1_D10: if (!(chunkVal & 0x01)) DDRB |= (1 << DDB2); else DDRB &= ~(1 << DDB2); break;
        case PRESET_W1_A0:  if (!(chunkVal & 0x01)) DDRC |= (1 << DDC0); else DDRC &= ~(1 << DDC0); break;

        // Width = 2
        case PRESET_W2_D4_D5:  DDRD = (DDRD & ~0x30) | ((~chunkVal & 0x03) << 4); break;
        case PRESET_W2_D6_D7:  DDRD = (DDRD & ~0xC0) | ((~chunkVal & 0x03) << 6); break;
        case PRESET_W2_D9_D10: DDRB = (DDRB & ~0x06) | ((~chunkVal & 0x03) << 1); break;
        case PRESET_W2_A0_A1:  DDRC = (DDRC & ~0x03) | (~chunkVal & 0x03);        break;

        // Width = 4
        case PRESET_W4_D4_D7:  DDRD = (DDRD & ~0xF0) | ((~chunkVal & 0x0F) << 4); break;
        case PRESET_W4_A0_A3:  DDRC = (DDRC & ~0x0F) | (~chunkVal & 0x0F);        break;
        case PRESET_W4_D9_D12: DDRB = (DDRB & ~0x1E) | ((~chunkVal & 0x0F) << 1); break;

        // Width = 8 (A0~A3 = MSB, D4~D7 = LSB)
        case PRESET_W8_A0_D7:
            DDRC = (DDRC & ~0x0F) | ((~chunkVal >> 4) & 0x0F);
            DDRD = (DDRD & ~0xF0) | ((~chunkVal & 0x0F) << 4);
            break;
    }
}

// Ultra Fast Port Reading
inline uint8_t SWP2P::_readDataChunk() {
    switch (_preset) {
        // Width = 1
        case PRESET_W1_D4:  return (PIND & (1 << PIND4)) ? 1 : 0;
        case PRESET_W1_D5:  return (PIND & (1 << PIND5)) ? 1 : 0;
        case PRESET_W1_D6:  return (PIND & (1 << PIND6)) ? 1 : 0;
        case PRESET_W1_D7:  return (PIND & (1 << PIND7)) ? 1 : 0;
        case PRESET_W1_D9:  return (PINB & (1 << PINB1)) ? 1 : 0;
        case PRESET_W1_D10: return (PINB & (1 << PINB2)) ? 1 : 0;
        case PRESET_W1_A0:  return (PINC & (1 << PINC0)) ? 1 : 0;

        // Width = 2
        case PRESET_W2_D4_D5:  return (PIND & 0x30) >> 4;
        case PRESET_W2_D6_D7:  return (PIND & 0xC0) >> 6;
        case PRESET_W2_D9_D10: return (PINB & 0x06) >> 1;
        case PRESET_W2_A0_A1:  return (PINC & 0x03);

        // Width = 4
        case PRESET_W4_D4_D7:  return (PIND & 0xF0) >> 4;
        case PRESET_W4_A0_A3:  return (PINC & 0x0F);
        case PRESET_W4_D9_D12: return (PINB & 0x1E) >> 1;

        // Width = 8 (A0~A3 = MSB, D4~D7 = LSB)
        case PRESET_W8_A0_D7:
            return ((PINC & 0x0F) << 4) | ((PIND & 0xF0) >> 4);
    }
    return 0;
}

// 1-Cycle Fast Release
inline void SWP2P::_dataRelease() {
    switch (_preset) {
        case PRESET_W1_D4:  DDRD &= ~(1 << DDD4); break;
        case PRESET_W1_D5:  DDRD &= ~(1 << DDD5); break;
        case PRESET_W1_D6:  DDRD &= ~(1 << DDD6); break;
        case PRESET_W1_D7:  DDRD &= ~(1 << DDD7); break;
        case PRESET_W1_D9:  DDRB &= ~(1 << DDB1); break;
        case PRESET_W1_D10: DDRB &= ~(1 << DDB2); break;
        case PRESET_W1_A0:  DDRC &= ~(1 << DDC0); break;

        case PRESET_W2_D4_D5:  DDRD &= ~0x30; break;
        case PRESET_W2_D6_D7:  DDRD &= ~0xC0; break;
        case PRESET_W2_D9_D10: DDRB &= ~0x06; break;
        case PRESET_W2_A0_A1:  DDRC &= ~0x03; break;

        case PRESET_W4_D4_D7:  DDRD &= ~0xF0; break;
        case PRESET_W4_A0_A3:  DDRC &= ~0x0F; break;
        case PRESET_W4_D9_D12: DDRB &= ~0x1E; break;

        case PRESET_W8_A0_D7:
            DDRC &= ~0x0F;
            DDRD &= ~0xF0;
            break;
    }
}

uint8_t SWP2P::_arbCycles() { return (8 + _dataWidth - 1) / _dataWidth; }

void SWP2P::begin(bool clkIsOutput, DataPreset preset, unsigned long clkFreq) {
    _clkIsOutput = clkIsOutput;
    _preset = preset;
    _clkFreq = clkFreq;

    switch (_preset) {
        case PRESET_W1_D4: case PRESET_W1_D5: case PRESET_W1_D6:
        case PRESET_W1_D7: case PRESET_W1_D9: case PRESET_W1_D10: case PRESET_W1_A0:
            _dataWidth = 1; break;
        case PRESET_W2_D4_D5: case PRESET_W2_D6_D7:
        case PRESET_W2_D9_D10: case PRESET_W2_A0_A1:
            _dataWidth = 2; break;
        case PRESET_W4_D4_D7: case PRESET_W4_A0_A3: case PRESET_W4_D9_D12:
            _dataWidth = 4; break;
        case PRESET_W8_A0_D7:
            _dataWidth = 8; break;
    }

    GPIOR0 = 0; // Hardware Flags Clear

    // CLK (D2 = PD2) -> Input
    DDRD &= ~(1 << DDD2);
    PORTD &= ~(1 << PORTD2);

    // BUSY (D3) & ACK (D8) Open-Drain Config (PORT=0, DDR=0 High-Z)
    PORTD &= ~(1 << PORTD3);
    DDRD &= ~(1 << DDD3);

    PORTB &= ~(1 << PORTB0);
    DDRB &= ~(1 << DDB0);

    // Initial Data Lines High-Z
    _dataRelease();

    _arbCyclesCached  = _arbCycles();
    _dataMaskCached   = (1 << _dataWidth) - 1;
    _arbShiftAmt      = 16 - _dataWidth;
    _txShiftAmt       = 8 - _dataWidth;

    if (_clkIsOutput) {
        DDRB |= (1 << DDB1);
        setupTimer1(_clkFreq);
    }

    // INT0 (CLK = D2) Any Logical Change
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    EICRA |= (1 << ISC00);
    EIMSK |= (1 << INT0);

    // INT1 (BUSY = D3) Any Logical Change
    EICRA &= ~((1 << ISC11) | (1 << ISC10));
    EICRA |= (1 << ISC10);
    EIMSK |= (1 << INT1);

    // PCINT0 (ACK = D8)
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

bool SWP2P::send(uint8_t destId, uint8_t data) {
    if (GET_FLAG(FLAG_IS_SENDING) || GET_FLAG(FLAG_IS_BUSY)) return false;
    _txDestId = destId;
    _txData = data;
    _txState = TX_PENDING;
    SET_FLAG(FLAG_IS_SENDING);
    return true;
}

bool SWP2P::available() { return _rxCount > 0; }

uint8_t SWP2P::read() {
    if (_rxCount == 0) return 0;
    uint8_t val = _rxFifo[_rxTail];
    _rxTail = (_rxTail + 1) & (SWP2P_FIFO_DEPTH - 1);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _rxCount--;
    }
    return val;
}

void SWP2P::_fifoPush(uint8_t val) {
    if (_rxCount >= SWP2P_FIFO_DEPTH) return;
    _rxFifo[_rxHead] = val;
    _rxHead = (_rxHead + 1) & (SWP2P_FIFO_DEPTH - 1);
    _rxCount++;
}

void SWP2P::_onClkEdge() {
    bool clkHigh = (PIND & (1 << PIND2)) != 0;

    if (clkHigh) {
        if (_txState == TX_PENDING) {
            _busyDriveLow();
            _txArbReg = ((uint16_t)_txDestId << 8) | _nodeId;
            _arbChunkCount = 2 * _arbCyclesCached;
            _txState = TX_ARB;

            _arbMyChunk = (_txArbReg >> _arbShiftAmt) & _dataMaskCached;
            _txArbReg <<= _dataWidth;
            _driveDataChunk(_arbMyChunk);
            _arbChunkCount--;
            return;
        }

        switch (_txState) {
            case TX_ARB: {
                if (_arbChunkCount == 0) {
                    _txDataReg = _txData;
                    _txDataChunkCount = _arbCyclesCached;
                    uint8_t chunk = (_txDataReg >> _txShiftAmt) & _dataMaskCached;
                    _txDataReg <<= _dataWidth;
                    _driveDataChunk(chunk);
                    _txDataChunkCount--;
                    _txState = TX_DATA;
                } else {
                    _arbMyChunk = (_txArbReg >> _arbShiftAmt) & _dataMaskCached;
                    _txArbReg <<= _dataWidth;
                    _driveDataChunk(_arbMyChunk);
                    _arbChunkCount--;
                }
                break;
            }
            case TX_DATA: {
                if (_txDataChunkCount == 0) {
                    _dataRelease();
                    _txState = TX_DONE;
                } else {
                    uint8_t chunk = (_txDataReg >> _txShiftAmt) & _dataMaskCached;
                    _txDataReg <<= _dataWidth;
                    _driveDataChunk(chunk);
                    _txDataChunkCount--;
                }
                break;
            }
            case TX_DONE:
                _busyRelease();
                CLR_FLAG(FLAG_IS_SENDING);
                _txState = TX_IDLE;
                break;
            default:
                break;
        }
    } else {
        if (_txState == TX_ARB) {
            uint8_t busVal = _readDataChunk();
            if ((_arbMyChunk & ~busVal) != 0) {
                _dataRelease();
                _busyRelease();
                CLR_FLAG(FLAG_IS_SENDING);
                _txState = TX_LOST;
            }
            return;
        }
        if (_txState != TX_IDLE) return;

        switch (_rxState) {
            case RX_ADDR: {
                uint8_t v = _readDataChunk();
                _rxAddrByte = (_rxAddrByte << _dataWidth) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    if (_rxAddrByte == _nodeId || _rxAddrByte == 0xFF) {
                        SET_FLAG(FLAG_IS_MY_PACKET);
                    } else {
                        CLR_FLAG(FLAG_IS_MY_PACKET);
                    }
                    _rxSrcByte = 0;
                    _rxChunkCount = _arbCyclesCached;
                    _rxState = RX_SRC;
                }
                break;
            }
            case RX_SRC: {
                uint8_t v = _readDataChunk();
                _rxSrcByte = (_rxSrcByte << _dataWidth) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    _rxDataByte = 0;
                    _rxChunkCount = _arbCyclesCached;
                    _rxState = RX_DATA;
                }
                break;
            }
            case RX_DATA: {
                uint8_t v = _readDataChunk();
                _rxDataByte = (_rxDataByte << _dataWidth) | v;
                _rxChunkCount--;
                if (_rxChunkCount == 0) {
                    if (GET_FLAG(FLAG_IS_MY_PACKET) && !GET_FLAG(FLAG_RX_CAPTURED)) {
                        _fifoPush(_rxDataByte);
                        SET_FLAG(FLAG_RX_CAPTURED);
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
    if (idle) {
        CLR_FLAG(FLAG_IS_BUSY);
        _ackRelease();
        CLR_FLAG(FLAG_RX_CAPTURED);
        _rxState = RX_IDLE;
        _rxAddrByte = 0;
        _rxSrcByte = 0;

        if (_txState == TX_LOST) {
            _txState = TX_PENDING;
            SET_FLAG(FLAG_IS_SENDING);
        }
    } else {
        SET_FLAG(FLAG_IS_BUSY);
        if (_txState == TX_IDLE) {
            _rxState = RX_ADDR;
            _rxAddrByte = 0;
            _rxChunkCount = _arbCyclesCached;
        }
    }
}

void SWP2P::_onAckEdge() {
    bool ackHigh = _ackRead();
    if (ackHigh) return;
}

ISR(INT0_vect)   { SWP2P::_onClkEdge(); }
ISR(INT1_vect)   { SWP2P::_onBusyEdge(); }
ISR(PCINT0_vect) { SWP2P::_onAckEdge(); }
