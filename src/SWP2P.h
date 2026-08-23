#ifndef SWP2P_H
#define SWP2P_H
#include <Arduino.h>
#define SWP2P_FIFO_DEPTH 4
class SWP2P {
public:
    // ISR(인터럽트)에서 접근할 수 있도록 public 선언
    static SWP2P* _instance;
    SWP2P(uint8_t nodeId);
    void begin(bool clkIsOutput, uint8_t* dataPins, uint8_t dataWidth, unsigned long clkFreq = 100000UL);
    bool send(uint8_t destId, uint8_t data);
    bool available();
    uint8_t read();
    bool isSending() const { return _isSending; }
    bool isBusy() const { return _isBusy; }
    // ISR 내부 호출용 핸들러 함수들
    void _onClkEdge();
    void _onBusyEdge();
    void _onAckEdge();

    // 핀별 포트 레지스터 캐시 (begin()에서 1회 계산, ISR에서는 포인터 역참조만 - digitalWrite류 금지)
    struct PinFast {
        volatile uint8_t* ddr;
        volatile uint8_t* port;
        volatile uint8_t* pin;
        uint8_t mask;
    };
private:
    uint8_t _nodeId;
    bool _clkIsOutput;
    uint8_t* _dataPins;
    uint8_t _dataWidth;
    unsigned long _clkFreq;
    volatile bool _isSending;
    volatile bool _isBusy;
    // Rx FIFO
    volatile uint8_t _rxFifo[SWP2P_FIFO_DEPTH];
    volatile uint8_t _rxHead;
    volatile uint8_t _rxTail;
    volatile uint8_t _rxCount;
    // Tx 상태: 대기(pending) -> 중재 -> 데이터 -> release -> done
    enum TxState : uint8_t { TX_IDLE, TX_PENDING, TX_ARB, TX_DATA, TX_RELEASE, TX_DONE, TX_LOST };
    volatile TxState _txState;
    uint8_t _txDestId;
    uint8_t _txData;
    volatile int8_t _arbChunkIdx;   // 중재: 상위 청크부터 0까지
    volatile uint8_t _arbMyChunk;   // 이번 중재 청크에서 내가 구동한 값 (반클럭 뒤 negedge에서 검증)
    volatile int8_t _txDataChunkIdx; // 데이터: 상위 청크부터 0까지 (WIDTH<8 다중클럭 전송용)
    // Rx 상태
    enum RxState : uint8_t { RX_IDLE, RX_ARB_WATCH, RX_ADDR, RX_DATA, RX_ACK };
    volatile RxState _rxState;
    volatile uint8_t _rxAddrByte;
    volatile uint8_t _rxDataByte;
    volatile int8_t _rxAddrChunkIdx; // 주소 수신 진행 중인 청크 인덱스
    volatile int8_t _rxDataChunkIdx; // 데이터 수신 진행 중인 청크 인덱스
    volatile bool _isMyPacket;
    volatile bool _rxCapturedThisFrame;
    void setupTimer1(unsigned long freq);
    void stopTimer1();

    PinFast _dataPinFast[8];
    void _cachePinFast(uint8_t pinNum, PinFast& out);

    // ---- WIDTH별 언롤 구현 (1/2/4/8) ----
    // 전부 static 함수: 멤버함수 포인터의 this-adjustment 오버헤드를 피하기 위해
    // PinFast 배열을 인자로 직접 받는다. ISR 핫패스이므로 함수 호출 자체도
    // begin()에서 한 번 고른 뒤 포인터로 고정, 매 엣지마다 분기(switch)하지 않는다.
    static void _drive_W1(PinFast* p, uint8_t chunkVal);
    static void _drive_W2(PinFast* p, uint8_t chunkVal);
    static void _drive_W4(PinFast* p, uint8_t chunkVal);
    static void _drive_W8(PinFast* p, uint8_t chunkVal);
    static uint8_t _read_W1(PinFast* p);
    static uint8_t _read_W2(PinFast* p);
    static uint8_t _read_W4(PinFast* p);
    static uint8_t _read_W8(PinFast* p);

    typedef void (*DriveFn)(PinFast*, uint8_t);
    typedef uint8_t (*ReadFn)(PinFast*);
    DriveFn _driveFn; // begin()에서 WIDTH에 맞게 1회 바인딩
    ReadFn  _readFn;

    // 얇은 래퍼 - 기존 호출부(_onClkEdge 등)는 이 두 줄만 통해서 부른다
    inline void _driveDataChunk(uint8_t chunkVal) { _driveFn(_dataPinFast, chunkVal); }
    inline uint8_t _readDataChunk() { return _readFn(_dataPinFast); }
    void _dataRelease();

    // ---- BUSY_N(D3=PORTD bit3) / ACK_N(D8=PORTB bit0) ----
    static inline void _busyDriveLow() { PORTD &= ~(1 << 3); DDRD |= (1 << 3); }
    static inline void _busyRelease()  { DDRD &= ~(1 << 3); PORTD &= ~(1 << 3); }
    static inline bool _busyRead()     { return (PIND & (1 << 3)) != 0; }
    static inline void _ackDriveLow()  { PORTB &= ~(1 << 0); DDRB |= (1 << 0); }
    static inline void _ackRelease()   { DDRB &= ~(1 << 0); PORTB &= ~(1 << 0); }
    static inline bool _ackRead()      { return (PINB & (1 << 0)) != 0; }

    void _fifoPush(uint8_t val);
    uint8_t _arbCycles() const; // ceil(8/_dataWidth) - begin() 이후 값 불변, 사용은 캐시된 멤버로

    // ---- ISR 핫패스 캐시 ----
    uint8_t _arbCyclesCached; // = ceil(8/_dataWidth)
    uint8_t _dataMaskCached;  // = (1 << _dataWidth) - 1
    uint8_t _dataTopBitCached; // W1/2/4는 내부에서 상수로 처리하므로 W8 한정으로만 실질 사용
};
#endif
