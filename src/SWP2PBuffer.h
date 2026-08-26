#ifndef SWP2P_BUFFER_H
#define SWP2P_BUFFER_H
#include <Arduino.h>
// 사용 예:
//   SWP2PBuffer<16> abcd;
//   while (1) {
//       node.buff(abcd, digitalRead(10));   // 비트 하나씩 밀어넣기 (8개 모이면 바이트 1개 완성)
//   }
//   node.sendBurst(0x02, abcd);             // buffer 전체(len()만큼) 전송
//   node.sendBurst(0x02, abcd, 4, 8);        // abcd[4]부터 8바이트만 전송
//   node.buffFree(abcd);                    // buffer 전체 비우기
//   node.buffFree(abcd, 2);                 // abcd의 2번 위치만 비우기(해당 값만 0, 뒤 데이터는 안 당겨짐)
//
// CAP는 컴파일타임 상수라야 SWP2P_MAX_BURST 체크도 컴파일타임에 걸 수 있음.
template <uint8_t CAP>
class SWP2PBuffer {
public:
    uint8_t data[CAP];
    uint8_t count;       // 지금까지 채워진 "완성된 바이트" 개수 (다음에 쓸 위치)
    SWP2PBuffer() : count(0), _bitAcc(0), _bitCnt(0) {
        for (uint8_t i = 0; i < CAP; i++) data[i] = 0;
    }
    // 비트 하나를 누산기에 밀어넣는다. MSB부터 채워서 8개가 모이면 바이트 완성.
    // buffer가 이미 꽉 찼으면(count==CAP) 조용히 무시한다(오버플로 방지).
    // 반환값: 이번 호출로 바이트 하나가 "방금 완성"되었으면 true (필요하면 사용자가 체크 가능)
    bool pushBit(bool bit) {
        _bitAcc = (_bitAcc << 1) | (bit ? 1 : 0);
        _bitCnt++;
        if (_bitCnt < 8) return false;
        _bitCnt = 0;
        return pushByte(_bitAcc);
    }
    // 바이트를 통째로 밀어넣는다. buffer가 꽉 찼으면 무시.
    // pushBit()로 모으던 중간 비트 누산기가 남아있었다면 함께 리셋한다 —
    // pushByte를 직접 호출한 시점부터는 "바이트 경계"로 간주하는 것이 맞기 때문.
    // (이렇게 해야 pushBit와 pushByte를 섞어 써도 다음 pushBit부터 비트 순서가
    //  뒤틀리지 않는다.)
    bool pushByte(uint8_t b) {
        if (count >= CAP) return false;
        data[count++] = b;
        _bitAcc = 0;
        _bitCnt = 0;
        return true;
    }
    uint8_t len() const { return count; }
    uint8_t capacity() const { return CAP; }
    bool full() const { return count >= CAP; }
    // buffer 전체를 비운다 (count=0, 비트 누산기도 리셋). 데이터 값 자체는 안 지워도 되지만
    // 혹시 남은 값 참조로 인한 혼동 방지 차 0으로 밀어준다.
    void clearAll() {
        count = 0;
        _bitAcc = 0;
        _bitCnt = 0;
        for (uint8_t i = 0; i < CAP; i++) data[i] = 0;
    }
    // buffer의 특정 위치(idx) 값 하나만 0으로 비운다. count(다음에 채울 위치)는
    // 건드리지 않는다 — buffer 중간을 비워도 뒤 데이터가 당겨지지 않는,
    // 딱 "그 자리만 지우는" 동작.
    //
    // *** 주의 ***
    // count는 그대로이므로, 이후 sendBurst(destId, buf)를 호출하면 idx 위치의
    // 0x00이 "실제 데이터"인 것처럼 count 길이만큼 그대로 전송된다.
    // 즉 이 함수는 "삭제"가 아니라 "해당 buffer 위치 값을 0으로 덮어쓰기"이며,
    // buffer 길이(count)나 전송 대상 바이트 수에는 전혀 영향을 주지 않는다.
    // 중간 요소를 진짜로 제거하고 싶다면(뒤 요소를 당기고 count-1) 별도 로직이
    // 필요하며, 이 클래스는 그 기능을 제공하지 않는다.
    void clearBuffer(uint8_t idx) {
        if (idx < CAP) data[idx] = 0;
    }
private:
    uint8_t _bitAcc;
    uint8_t _bitCnt;
};
#endif
