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
    // TX_PENDING: send()가 호출된 직후, 아직 실제 BUSY_N을 구동하지 않은 상태.
    //             다음 CLK posedge에서 _onClkEdge()가 BUSY_N 하강과 첫 주소 청크
    //             구동을 "동시에" 실행하도록 하기 위한 동기화용 중간 상태.
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
    // 핀별 포트 레지스터 캐시 (begin()에서 1회 계산, ISR에서는 포인터 역참조만 - digitalWrite류 금지)
    struct PinFast {
        volatile uint8_t* ddr;
        volatile uint8_t* port;
        volatile uint8_t* pin;
        uint8_t mask;
    };
    PinFast _dataPinFast[8];
    void _cachePinFast(uint8_t pinNum, PinFast& out);
    // 데이터선 open-drain 헬퍼 (임의 핀 배열 -> 범용 API 사용, DATA_BUS는 런타임 핀이라 포인터 방식 불가피)
    void _driveDataChunk(uint8_t chunkVal);   // WIDTH비트 동시 구동/release
    uint8_t _readDataChunk();
    void _dataRelease();

    // ---- BUSY_N(D3=PORTD bit3) / ACK_N(D8=PORTB bit0) ----
    // 이 두 핀은 #define으로 고정된 컴파일타임 상수이므로, DATA_BUS처럼 런타임 포인터
    // 캐시(PinFast)를 거치지 않고 레지스터를 직접 컴파일타임 상수로 조작한다.
    // 이러면 (1) 함수 호출/리턴 오버헤드가 인라인으로 사라지고, (2) 컴파일러가 런타임
    // 포인터 역참조 대신 컴파일타임 비트 위치를 알고 있으므로 훨씬 짧은 코드를 낼 수 있다.
    // (DATA_BUS는 begin()에서 사용자가 임의로 핀을 고를 수 있어 이 최적화를 적용할 수 없다.)
    static inline void _busyDriveLow() { PORTD &= ~(1 << 3); DDRD |= (1 << 3); }
    static inline void _busyRelease()  { DDRD &= ~(1 << 3); PORTD &= ~(1 << 3); }
    static inline bool _busyRead()     { return (PIND & (1 << 3)) != 0; }
    static inline void _ackDriveLow()  { PORTB &= ~(1 << 0); DDRB |= (1 << 0); }
    static inline void _ackRelease()   { DDRB &= ~(1 << 0); PORTB &= ~(1 << 0); }
    static inline bool _ackRead()      { return (PINB & (1 << 0)) != 0; }

    void _fifoPush(uint8_t val);
    uint8_t _arbCycles() const; // ceil(8/_dataWidth) - begin() 이후 값 불변, 사용은 캐시된 멤버로

    // ---- ISR 핫패스 캐시 (begin()에서 1회 계산, 이후 절대 재계산 안 함) ----
    // _dataWidth는 begin() 호출 뒤로는 바뀌지 않으므로, 매 CLK 엣지마다
    // 나눗셈/시프트를 다시 하는 대신 미리 계산해둔 값만 읽는다.
    // (AVR은 나눗셈 명령이 없고, 가변 시프트도 시프트량만큼 반복하는 소프트웨어 루프라
    //  둘 다 느리다.)
    uint8_t _arbCyclesCached; // = ceil(8/_dataWidth)
    uint8_t _dataMaskCached;  // = (1 << _dataWidth) - 1
    uint8_t _dataTopBitCached; // = 1 << (_dataWidth - 1) : _driveDataChunk의 시작 비트 (가변 시프트를 1회로 제한)
};
#endif
