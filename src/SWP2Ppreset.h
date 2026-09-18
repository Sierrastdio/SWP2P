/**********************************************************************************************************
 * SWP2P
 * Copyright (c) 2026 Sierrastdio
 *
 * SWP2P is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SWP2P is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 *            <http://www.gnu.org/licenses/>.
 *********************************************************************************************************/

// ============================================================
// SWP2PPreset.h
// ------------------------------------------------------------
// DataPreset(핀 구성) 관련 코드를 SWP2P.h에서 분리해 모아둔 파일.
// - enum DataPreset 정의
// - 프리셋별 폭(WIDTH)/파생 상수를 담은 SWP2PPresetTraits<PRESET>
// - 프리셋별 핀 조작(_driveDataChunk/_readDataChunk/_dataRelease)을
//   담당하는 free 함수 swp2p_driveDataChunk<PRESET>() 등
//
// SWP2P.h의 SWP2P<PRESET> 템플릿 클래스는 이 파일만 include해서
// 위 free 함수/트레이트를 그대로 호출한다. 새 프리셋을 추가하고
// 싶으면 이 파일만 건드리면 된다 (SWP2P.h 본체는 손댈 필요 없음).
// ============================================================

#ifndef SWP2P_PRESET_H
#define SWP2P_PRESET_H

#include <avr/io.h>

enum DataPreset : uint8_t {
    PRESET_W1_D4 = 0, PRESET_W1_D5, PRESET_W1_D6, PRESET_W1_D7, PRESET_W1_D9, PRESET_W1_D10, PRESET_W1_A0,
    PRESET_W2_D4_D5, PRESET_W2_D6_D7, PRESET_W2_D9_D10, PRESET_W2_A0_A1,
    PRESET_W4_D4_D7, PRESET_W4_A0_A3, PRESET_W4_D9_D12,
    PRESET_W8_A0_D7
};

/* Arduino pinout
 *
 * D0~D7:   PD0~PD7
 * D8~D13:  PB0~PB5
 * A0~A5:   PC0~PC5
 *
 */

// ---- 프리셋별 파생 상수 ----
// DATA_WIDTH: 프리셋이 쓰는 데이터선 개수(= 클럭 한 번에 나르는 비트 수)
// ARB_CYCLES: 8비트(1바이트)를 이 폭으로 나누어 보내는 데 필요한 사이클 수
// DATA_MASK : 하위 DATA_WIDTH비트만 남기는 마스크 (2^WIDTH - 1)
// ARB_SHIFT : 중재용 16비트 레지스터에서 이번 청크를 뽑아내는 시프트량
// TX_SHIFT  : 데이터/LEN 전송용 8비트 레지스터에서 이번 청크를 뽑아내는 시프트량
template <DataPreset PRESET>
struct SWP2PPresetTraits {
    static constexpr uint8_t DATA_WIDTH =
        (PRESET >= PRESET_W8_A0_D7) ? 8 :
        (PRESET >= PRESET_W4_D4_D7) ? 4 :
        (PRESET >= PRESET_W2_D4_D5) ? 2 : 1;

    static constexpr uint8_t ARB_CYCLES = (8 + DATA_WIDTH - 1) / DATA_WIDTH;

    // [2^N - 1 원리] (1 << DATA_WIDTH) - 1: 하위 N비트(DATA_WIDTH 크기)만 1로 채워진 거름망 마스크 생성
    // N=1: (1<<1)-1 = 0b00000001 (0x01)
    // N=2: (1<<2)-1 = 0b00000011 (0x03)
    // N=4: (1<<4)-1 = 0b00001111 (0x0F)
    // N=8: (1<<8)-1 = 0b11111111 (0xFF)
    static constexpr uint8_t DATA_MASK = (1 << DATA_WIDTH) - 1;

    static constexpr uint8_t ARB_SHIFT = 16 - DATA_WIDTH;
    static constexpr uint8_t TX_SHIFT  = 8 - DATA_WIDTH;
};

// ---- 프리셋별 핀 조작 (컴파일타임 특수화, 항상 인라인) ----

// 데이터 핀 방향(DDR)을 통해 chunkVal(하위 DATA_WIDTH비트)을 open-drain으로 구동.
// 1(하이임피던스) : 입력으로 풀어줌 -> 풀업에 의해 버스가 1로 보임
// 0(구동)         : 출력으로 전환, PORT는 항상 0으로 유지되어 있으므로 0을 구동
template <DataPreset PRESET>
static inline void swp2p_driveDataChunk(uint8_t chunkVal) __attribute__((always_inline));

template <DataPreset PRESET>
static inline void swp2p_driveDataChunk(uint8_t chunkVal) {
    // [DDR_reg] &= [BitMask_Clear] | [DataMask_Set] => 지정된 데이터 핀 방향 제어 (1:출력=0구동, 0:입력=하이임피던스)
    if constexpr (PRESET == PRESET_W1_D4) { DDRD = (DDRD & ~(1 << DDD4)) | ((~chunkVal & 0x01) << 4); }
    else if constexpr (PRESET == PRESET_W1_D5) { DDRD = (DDRD & ~(1 << DDD5)) | ((~chunkVal & 0x01) << 5); }
    else if constexpr (PRESET == PRESET_W1_D6) { DDRD = (DDRD & ~(1 << DDD6)) | ((~chunkVal & 0x01) << 6); }
    else if constexpr (PRESET == PRESET_W1_D7) { DDRD = (DDRD & ~(1 << DDD7)) | ((~chunkVal & 0x01) << 7); }
    else if constexpr (PRESET == PRESET_W1_D9) { DDRB = (DDRB & ~(1 << DDB1)) | ((~chunkVal & 0x01) << 1); }
    else if constexpr (PRESET == PRESET_W1_D10){ DDRB = (DDRB & ~(1 << DDB2)) | ((~chunkVal & 0x01) << 2); }
    else if constexpr (PRESET == PRESET_W1_A0) { DDRC = (DDRC & ~(1 << DDC0)) | (~chunkVal & 0x01); }
    else if constexpr (PRESET == PRESET_W2_D4_D5) { DDRD = (DDRD & ~0x30) | ((~chunkVal & 0x03) << 4); }
    else if constexpr (PRESET == PRESET_W2_D6_D7) { DDRD = (DDRD & ~0xC0) | ((~chunkVal & 0x03) << 6); }
    else if constexpr (PRESET == PRESET_W2_D9_D10){ DDRB = (DDRB & ~0x06) | ((~chunkVal & 0x03) << 1); }
    else if constexpr (PRESET == PRESET_W2_A0_A1) { DDRC = (DDRC & ~0x03) | (~chunkVal & 0x03); }
    else if constexpr (PRESET == PRESET_W4_D4_D7) { DDRD = (DDRD & ~0xF0) | ((~chunkVal & 0x0F) << 4); }
    else if constexpr (PRESET == PRESET_W4_A0_A3) { DDRC = (DDRC & ~0x0F) | (~chunkVal & 0x0F); }
    else if constexpr (PRESET == PRESET_W4_D9_D12) { DDRB = (DDRB & ~0x1E) | ((~chunkVal & 0x0F) << 1); }
    else if constexpr (PRESET == PRESET_W8_A0_D7) {
        DDRC = (DDRC & ~0x0F) | (~chunkVal & 0x0F);
        DDRD = (DDRD & ~0xF0) | ((~chunkVal & 0xF0));
    }
}

// 데이터 핀 상태를 읽어 하위 DATA_WIDTH비트로 정렬해 반환 (중재 되읽기 / 데이터 수신용)
template <DataPreset PRESET>
static inline uint8_t swp2p_readDataChunk() __attribute__((always_inline));

template <DataPreset PRESET>
static inline uint8_t swp2p_readDataChunk() {
    uint8_t val = 0;
    // [PIN_reg] >> [Shift] & [BitMask] => 데이터 핀 상태만 0~N 비트 크기로 정렬하여 수신
    if constexpr (PRESET == PRESET_W1_D4)   val = (PIND >> PIND4) & 0x01;
    else if constexpr (PRESET == PRESET_W1_D5)  val = (PIND >> PIND5) & 0x01;
    else if constexpr (PRESET == PRESET_W1_D6)  val = (PIND >> PIND6) & 0x01;
    else if constexpr (PRESET == PRESET_W1_D7)  val = (PIND >> PIND7) & 0x01;
    else if constexpr (PRESET == PRESET_W1_D9)  val = (PINB >> PINB1) & 0x01;
    else if constexpr (PRESET == PRESET_W1_D10) val = (PINB >> PINB2) & 0x01;
    else if constexpr (PRESET == PRESET_W1_A0)  val = (PINC >> PINC0) & 0x01;
    else if constexpr (PRESET == PRESET_W2_D4_D5) val = (PIND & 0x30) >> 4;
    else if constexpr (PRESET == PRESET_W2_D6_D7) val = (PIND & 0xC0) >> 6;
    else if constexpr (PRESET == PRESET_W2_D9_D10)val = (PINB & 0x06) >> 1;
    else if constexpr (PRESET == PRESET_W2_A0_A1) val = (PINC & 0x03);
    else if constexpr (PRESET == PRESET_W4_D4_D7) val = (PIND & 0xF0) >> 4;
    else if constexpr (PRESET == PRESET_W4_A0_A3) val = (PINC & 0x0F);
    else if constexpr (PRESET == PRESET_W4_D9_D12) val = (PINB & 0x1E) >> 1;
    else if constexpr (PRESET == PRESET_W8_A0_D7) val = (PINC & 0x0F) | (PIND & 0xF0);
    return val;
}

// 데이터 핀 전부를 입력(High-Z)으로 해제 (송신 종료 시 라인 놓아주기)
template <DataPreset PRESET>
static inline void swp2p_dataRelease() __attribute__((always_inline));

template <DataPreset PRESET>
static inline void swp2p_dataRelease() {
    // [DDR_reg] &= ~(BitMask) => 데이터 핀을 모두 입력(High-Z)으로 해제
    if constexpr (PRESET == PRESET_W1_D4) DDRD &= ~(1 << DDD4);
    else if constexpr (PRESET == PRESET_W1_D5) DDRD &= ~(1 << DDD5);
    else if constexpr (PRESET == PRESET_W1_D6) DDRD &= ~(1 << DDD6);
    else if constexpr (PRESET == PRESET_W1_D7) DDRD &= ~(1 << DDD7);
    else if constexpr (PRESET == PRESET_W1_D9) DDRB &= ~(1 << DDB1);
    else if constexpr (PRESET == PRESET_W1_D10) DDRB &= ~(1 << DDB2);
    else if constexpr (PRESET == PRESET_W1_A0) DDRC &= ~(1 << DDC0);
    else if constexpr (PRESET == PRESET_W2_D4_D5) DDRD &= ~0x30;
    else if constexpr (PRESET == PRESET_W2_D6_D7) DDRD &= ~0xC0;
    else if constexpr (PRESET == PRESET_W2_D9_D10) DDRB &= ~0x06;
    else if constexpr (PRESET == PRESET_W2_A0_A1) DDRC &= ~0x03;
    else if constexpr (PRESET == PRESET_W4_D4_D7) DDRD &= ~0xF0;
    else if constexpr (PRESET == PRESET_W4_A0_A3) DDRC &= ~0x0F;
    else if constexpr (PRESET == PRESET_W4_D9_D12) DDRB &= ~0x1E;
    else if constexpr (PRESET == PRESET_W8_A0_D7) { DDRC &= ~0x0F; DDRD &= ~0xF0; }
}

#endif
