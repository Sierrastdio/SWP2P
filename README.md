# SWP2P (Software Peer-to-Peer)

A masterless peer-to-peer communication library for AVR (Arduino) nodes.
Unlike SPI/I2C/CAN, where slave-to-slave traffic has to be relayed through a master,
SWP2P lets nodes talk to each other directly, without any node acting as a bus master.

## Features

- **Masterless architecture**: One node may drive the CLK line, but that node does not act as a bus master. All nodes participate in send/receive as equals.
- **Configurable data width**: Choose 1/2/4/8 data lines (`DataPreset`). More lines means more bits per clock edge and higher throughput.
- **CLK/BUSY/ACK based protocol**: Only a minimal set of control lines (BUSY, ACK) is needed, supporting roughly 10 nodes on the bus by design.
- **Fixed-ID CLK slot arbitration**: Instead of bit-level open-drain arbitration, transmission turns are ordered by fixed-ID CLK slots after the bus returns to idle. When BUSY is idle, the slot counter resets, and each node inspects BUSY on its slot. Only if BUSY is still HIGH does that node assert BUSY LOW and begin transmission. (Round-Robin fairness can be optionally enabled).
- **Full push-pull data frame**: Because only the node that wins the slot transmits, the entire frame—from header through LEN and DATA—is driven push-pull. This significantly sharpens edge transitions and allows higher CLK rates compared to open-drain.
- **Single-byte and burst transfers**: `send()` uses the original fast path (no LEN field); `sendBurst()` adds a LEN field to transfer multiple bytes in one frame.
- **Real ACK with timeout**: The sender doesn't declare success just because it finished driving data — it waits for the addressed receiver to actively drive the ACK line low, with a bounded timeout (`isAckFailed()` reports failed transfers).
- **Sender ID delivered with every received byte**: The RX FIFO stores a source-ID alongside each data byte, so a node reading from multiple peers can tell them apart.
- **Interrupt-driven, non-blocking**: Uses INT0 (CLK), INT1 (BUSY), and Timer1 Input Capture (ACK edge detection), plus Timer1 output-compare for automatic CLK generation, to minimize CPU overhead. ISRs are bound at compile time via a macro (no function-pointer indirection).
- **RX FIFO + user-friendly buffer API**: `SWP2PBuffer<CAP>` lets you accumulate data bit-by-bit or byte-by-byte, then send it whole or as a sub-range.

## Hardware Layout

| Signal | Pin (default) | Description |
|---|---|---|
| CLK | D2 (INT0) | Communication clock. Driven by one node (not a master) or an external clock source |
| BUSY | D3 (INT1) | Bus-busy indicator. Watched on both edges (any logical change) so the idle transition is never missed |
| ACK | D8 / PB0 (Timer1 Input Capture, ICP1) | Actively driven low by the addressed receiver to confirm reception |
| Data line(s) | D4–D7 / D9,D10 / A0–A3, etc., depending on preset | Selected via `DataPreset` |

This branch drives the entire data frame (HEADER, LEN, and payload DATA) push-pull after a sender claims its CLK slot. BUSY and ACK remain open-drain. External pull-ups are required for the control lines (BUSY, ACK) and recommended for data lines to keep lines stable when idle.

## Installation

Copy `SWP2P.h`, `SWP2P.cpp`, `SWP2Ppreset.h`, `SWP2Pbuffer.h` into a `Arduino/libraries/SWP2P/` folder in your sketchbook, then `#include "SWP2P.h"` from the Arduino IDE.

## Quick Start

```cpp
#include "SWP2P.h"

// Node using 1 data line (D4), node ID = 1
SWP2P<PRESET_W1_D4> node(1);

// Bind the ISRs for this preset (do this exactly once, at file scope)
SWP2P_BIND_ISRS(PRESET_W1_D4);

void setup() {
    // Whether this node drives CLK, and the clock frequency (Hz)
    node.begin(true, 1000UL);
}

void loop() {
    // Send 1 byte to node 2
    node.send(2, 0x42);

    // Check for received data
    uint8_t data, srcId;
    if (node.read(data, srcId)) {
        // data came from node `srcId`
    }
}
```

## API

### Node creation / initialization

```cpp
SWP2P<DataPreset> node(uint8_t nodeId);
void node.begin(bool clkIsOutput, unsigned long clkFreq = 100000UL);

// Once per preset, at file scope (not inside a function):
SWP2P_BIND_ISRS(DataPreset);
```

- Use node IDs 0–62; `0x3F` (63) is reserved for broadcast. The constructor masks IDs to six bits and maps reserved ID 63 to 62.
- If `clkIsOutput = true`, this node automatically drives CLK using Timer1.
- `SWP2P_BIND_ISRS(preset)` expands to the `ISR(INT0_vect)`, `ISR(INT1_vect)`, and `ISR(TIMER1_CAPT_vect)` definitions bound to that specific `SWP2P<preset>` specialization. Call it exactly once for whichever preset you actually use.

### Sending

```cpp
bool node.send(uint8_t destId, uint8_t data);
bool node.sendBurst(uint8_t destId, const uint8_t* buf, uint8_t len);
```

- `send()` is the single-byte fast path (no LEN field).
- `sendBurst()` sets the BURST bit after the 6-bit DEST and 6-bit SRC fields when `len > 1`, then transmits a compact LEN field containing `len - 2`. LEN width is derived from `SWP2P_MAX_BURST` and rounded to whole preset-width chunks. `len` must be between 1 and `SWP2P_MAX_BURST` (default 16).
- Returns `false` if a transfer is already in progress (`isSending()`) or the bus is busy (`isBusy()`).
- Use `SWP2P<PRESET>::BROADCAST` as `destId` to broadcast to all nodes.
- A send only completes successfully once the addressed receiver actively acknowledges it (see **Status checks** / `isAckFailed()`).

### Receiving

```cpp
bool node.available();
bool node.read(uint8_t& data, uint8_t& srcId);
uint8_t node.readBytes(uint8_t* outBuf, uint8_t* srcBuf, uint8_t maxLen);
bool node.peek(uint8_t& data, uint8_t& srcId);
void node.flush();
```

- `read()` / `peek()` always return both the data byte and the sender's node ID together — `outBuf[i]` and `srcBuf[i]` in `readBytes()` correspond 1:1, since consecutive queued bytes may come from different senders.
- Received data is queued in an internal FIFO (`SWP2P_FIFO_DEPTH`, default 16, must be a power of 2). Only packets addressed to this node (or broadcast packets) are captured into the FIFO.

### Status checks

```cpp
bool node.isSending();
bool node.isBusy();
bool node.isAckFailed();
```

- `isAckFailed()` reports whether the most recent `send()`/`sendBurst()` timed out waiting for the receiver's ACK. Reading it clears the flag, so check it right after a send whose result you care about.

### Buffer API (`SWP2PBuffer<CAP>`)

Use this to accumulate multiple bytes bit-by-bit or byte-by-byte before sending them together as a burst.

```cpp
SWP2PBuffer<16> buf;

// Push one bit at a time (a byte is completed every 8 bits)
node.buff(buf, digitalRead(10));

// Push a whole byte at once
node.buff(buf, someByte, true);

// Send the entire buffer (buf.len() bytes)
node.sendBurst(destId, buf);

// Send only 8 bytes starting at buf[4]
node.sendBurst(destId, buf, 4, 8);

// Clear the entire buffer
node.buffFree(buf);

// Clear only buf[2] (count stays the same, later data is not shifted)
node.buffFree(buf, 2);
```

- `CAP` in `SWP2PBuffer<CAP>` is a compile-time constant; a compile error is raised if `CAP > SWP2P_MAX_BURST`.
- `buffFree(buf, idx)` does not delete the element — it overwrites that position with 0. It does not affect `count` (the buffer length), so a subsequent `sendBurst(dest, buf)` call will still transmit the 0x00 at that position as if it were real data.

### `DataPreset` (data line configuration)

| Preset | Data line(s) | Width |
|---|---|---|
| `PRESET_W1_D4`–`PRESET_W1_D7`, `D10`, `A0` | 1 | 1 bit |
| `PRESET_W1_D9` | 1 (D9) | 1 bit (Slave only, conflicts with CLK master D9) |
| `PRESET_W2_D4_D5`, `PRESET_W2_D6_D7`, `PRESET_W2_A0_A1` | 2 | 2 bits |
| `PRESET_W2_D9_D10` | 2 (D9,D10) | 2 bits (Slave only, conflicts with CLK master D9) |
| `PRESET_W4_D4_D7` (Default), `PRESET_W4_A0_A3` | 4 | 4 bits |
| `PRESET_W4_D9_D12` | 4 (D9~D12) | 4 bits (Slave only, conflicts with CLK master D9) |
| `PRESET_W8_A0_D7` | 8 | 8 bits |

A wider data width carries more bits per clock edge, reducing the number of transfer cycles — but in practice, ISR execution time becomes the bottleneck, so the maximum achievable clock frequency can actually be lower with wider widths (measured internally: roughly 40kHz at WIDTH=1 vs. roughly 29kHz at WIDTH=8). Consider this trade-off when choosing a preset for your use case.

## Protocol Overview

1. **HEADER (Slot-based allocation)**: When BUSY returns idle, the slot counter resets to 0. On each CLK rising edge, if `_slotCounter == _nodeId` and BUSY is HIGH, the node asserts BUSY LOW and starts driving the packed header `DEST(6) + SRC(6) + BURST(1)` push-pull. Other nodes observe BUSY going LOW and enter RX state without collision.
2. **LEN (burst frames only)**: The value `len - 2` is encoded using the minimum bit width required by `SWP2P_MAX_BURST`, padded to whole preset-width chunks, driven push-pull. This step is skipped for single-byte transfers.
3. **DATA**: 1 byte (single transfer) or `len` bytes (burst) are transmitted in sequence push-pull.
4. **ACK**: The sender releases the data line(s) but keeps BUSY asserted while it waits a bounded number of clock edges for the addressed receiver to drive ACK low. If the ACK arrives in time, the send is marked successful; otherwise it times out and `isAckFailed()` will report `true` on the next check. Either way, BUSY is released once the wait ends so other nodes can use the bus.

Receiving nodes only capture packets addressed to them (or broadcast packets) into the FIFO, and only the addressed receiver drives ACK; other packets/nodes are tracked for line-state purposes only and otherwise ignored.

## Design Constraints / Notes

- Control lines (BUSY, ACK) require external pull-up resistors as they operate open-drain. Data lines are driven push-pull during transmission, but pull-ups are recommended for floating state during bus idle.
- Node IDs are limited to 0–62; 63 (`SWP2P_BROADCAST`) is reserved for broadcast. The 13-bit packed header takes 13 clocks at W1, 7 at W2, 4 at W4, and 2 at W8. Compared with the old 16-bit header, W1 saves 3 clocks and W2 saves 1; W4 and W8 use the same number of whole chunks.
- `SWP2P_MAX_BURST` is limited to 255 by the 8-bit API length. The LEN field width is derived at compile time; increasing the limit also increases required LEN bits and TX buffer RAM. `SWP2PBuffer<CAP>` with `CAP > SWP2P_MAX_BURST` fails at compile time and is not split automatically.
- This packed-header/LEN format is not wire-compatible with older releases; every node on a bus must use the same format.
- The maximum usable clock frequency depends on your environment (pull-up resistor values, wiring, WIDTH setting); measuring and tuning it empirically is recommended.

## File Overview

- `SWP2P.h` — `SWP2PBase` (shared static state) + `SWP2P<DataPreset>` template class (compile-time specialization per pin configuration) + `SWP2P_BIND_ISRS` macro
- `SWP2P.cpp` — static member definitions, Timer1 setup/teardown, shared FIFO push logic
- `SWP2PBuffer.h` — user-friendly buffer template class for accumulating data bit-by-bit or byte-by-byte


***


# SWP2P (Software Peer-to-Peer)

AVR(아두이노) 기반 마스터 없는 대등(peer-to-peer) 노드 간 통신 라이브러리입니다.
SPI/I2C/CAN처럼 마스터를 거쳐 슬레이브 ↔ 슬레이브 통신을 중계하는 오버헤드 없이,
노드끼리 직접 통신하는 것을 목표로 합니다.

## 특징

- **마스터 없는 구조**: CLK을 출력하는 노드가 있을 수는 있지만, 그 노드가 버스의 주인(마스터) 역할을 하지 않습니다. 모든 노드가 대등하게 송수신에 참여합니다.
- **가변 데이터 폭**: 데이터선을 1/2/4/8개 중에서 선택할 수 있습니다 (`DataPreset`). 선 개수가 늘어날수록 사이클당 더 많은 비트를 실어 전송 속도를 높일 수 있습니다.
- **CLK/BUSY/ACK 3선 기반 프로토콜**: 최소한의 제어선(BUSY, ACK)만으로 다수 노드(설계 목표 약 10개) 연결을 지원합니다.
- **고정 ID별 CLK 슬롯 송신권 결정**: 헤더 비트 중재 대신 버스 idle 복귀 후 CLK 상승 에지마다 증가하는 슬롯 카운터를 기반으로 송신 시점을 순서화합니다. 각 노드는 자기 슬롯에서 BUSY 핀이 HIGH인 경우에만 버스를 점유(LOW)하고 송신을 시작하므로 데이터선 충돌이 원천 방지됩니다(필요시 라운드로빈 공정성 활성화 가능).
- **데이터 프레임 전체 푸시풀(Push-Pull) 구동**: 슬롯을 점유한 단일 송신자만 프레임을 전송하므로, 헤더부터 LEN/DATA까지 프레임 전체를 푸시풀로 구동합니다. 오픈드레인 방식 대비 신호 상승 시간이 대폭 개선되어 더 높은 CLK 주파수에서도 안정적인 통신이 가능합니다.
- **단일 바이트 / 버스트 전송 모두 지원**: `send()`는 기존 빠른 경로를 그대로 사용하고, `sendBurst()`는 길이(LEN) 필드를 추가로 실어 여러 바이트를 한 번에 전송합니다.
- **실제 ACK 확인 + 타임아웃**: 데이터를 다 보냈다고 곧바로 성공 처리하지 않고, 목적지 노드가 실제로 ACK 라인을 LOW로 구동하는지 제한된 시간 동안 기다립니다(`isAckFailed()`로 실패 여부 조회 가능).
- **수신 바이트마다 발신자 ID 동반 제공**: RX FIFO에 데이터 바이트와 발신자 ID가 함께 쌓여, 여러 상대와 통신하는 노드도 출처를 구분할 수 있습니다.
- **인터럽트 기반, 논블로킹**: INT0(CLK), INT1(BUSY), Timer1 Input Capture(ACK 엣지 감지), Timer1 출력비교(CLK 자동 출력)를 활용해 CPU 부담을 최소화합니다. ISR은 함수 포인터 간접호출 없이 매크로를 통해 컴파일타임에 바인딩됩니다.
- **수신 FIFO + 사용자 친화 버퍼 API**: `SWP2PBuffer<CAP>`로 비트/바이트 단위로 데이터를 모으고, 그대로 혹은 부분 범위만 잘라서 전송할 수 있습니다.

## 하드웨어 구성

| 신호 | 핀 (기본) | 설명 |
|---|---|---|
| CLK | D2 (INT0) | 통신 클럭. 한 노드가 출력(마스터 아님) 또는 외부 클럭 |
| BUSY | D3 (INT1) | 버스 점유 여부. 양쪽 엣지(Any Logical Change)를 모두 감지해 idle 복귀 시점을 놓치지 않음 |
| ACK | D8 / PB0 (Timer1 Input Capture, ICP1) | 목적지 노드가 실제로 LOW 구동하여 수신을 확인 |
| 데이터선 | Preset에 따라 D4~D7 / D9,D10 / A0~A3 등 | `DataPreset`으로 선택 |

이 실험 브랜치에서는 슬롯을 획득한 송신 노드가 헤더부터 LEN, DATA까지 프레임 전체를 push-pull로 구동합니다. BUSY와 ACK는 계속 open-drain입니다. 제어선(BUSY, ACK)에는 외부 풀업 저항이 필수이며, 데이터선도 idle 시 플로팅 방지를 위해 풀업 저항 사용을 권장합니다. 슬롯 선점을 통해 충돌 없는 단독 전송이 보장됩니다.

## 설치

`SWP2P.h`, `SWP2P.cpp`, `SWP2Ppreset.h`, `SWP2Pbuffer.h` 세 파일을 `Arduino/libraries/SWP2P/` 폴더에 넣고 아두이노 IDE에서 `#include "SWP2P.h"`로 사용합니다.

## 빠른 시작

```cpp
#include "SWP2P.h"

// 데이터선 1개(D4)를 사용하는 노드, 노드 ID = 1
SWP2P<PRESET_W1_D4> node(1);

// 이 preset에 대한 ISR 바인딩 (파일 스코프에서 딱 한 번만 호출)
SWP2P_BIND_ISRS(PRESET_W1_D4);

void setup() {
    // 이 노드가 CLK을 출력할지 여부, 클럭 주파수(Hz)
    node.begin(true, 1000UL);
}

void loop() {
    // 노드 2번으로 1바이트 전송
    node.send(2, 0x42);

    // 수신 확인
    uint8_t data, srcId;
    if (node.read(data, srcId)) {
        // data는 srcId 노드로부터 온 값
    }
}
```

## API

### 노드 생성 / 초기화

```cpp
SWP2P<DataPreset> node(uint8_t nodeId);
void node.begin(bool clkIsOutput, unsigned long clkFreq = 100000UL);

// 사용하는 preset마다 파일 스코프에서 한 번:
SWP2P_BIND_ISRS(DataPreset);
```

- 노드 ID는 0~62를 사용하고 `0x3F`(63)은 브로드캐스트 전용입니다. 생성자는 ID를 6비트로 마스킹하며 예약값 63은 62로 조정합니다.
- `clkIsOutput = true`이면 Timer1을 이용해 이 노드가 CLK을 자동 출력합니다.
- `SWP2P_BIND_ISRS(preset)`는 해당 `SWP2P<preset>` 특수화에 바인딩된 `ISR(INT0_vect)`, `ISR(INT1_vect)`, `ISR(TIMER1_CAPT_vect)` 정의로 전개됩니다. 실제 사용하는 preset에 대해 정확히 한 번만 호출하면 됩니다.

### 전송

```cpp
bool node.send(uint8_t destId, uint8_t data);
bool node.sendBurst(uint8_t destId, const uint8_t* buf, uint8_t len);
```

- `send()`는 1바이트 전용 빠른 경로(LEN 필드 없음)입니다.
- `sendBurst()`는 `len > 1`일 때 6비트 DEST와 6비트 SRC 뒤에 BURST 비트를 붙이고, `len-2`를 담은 압축 LEN 필드를 전송합니다. LEN의 유효 비트 수는 `SWP2P_MAX_BURST`에서 컴파일타임 계산 후 프리셋 폭에 맞춰 청크 단위로 올림합니다. `len`은 1~`SWP2P_MAX_BURST`(기본 16) 범위여야 합니다.
- 이미 전송 중이거나(`isSending()`) 버스가 사용 중이면(`isBusy()`) `false`를 반환합니다.
- `destId`로 `SWP2P<PRESET>::BROADCAST`를 사용하면 모든 노드에 브로드캐스트됩니다.
- 목적지 노드가 실제로 ACK을 구동해야만 전송이 최종 성공으로 간주됩니다(아래 **상태 확인** / `isAckFailed()` 참고).

### 수신

```cpp
bool node.available();
bool node.read(uint8_t& data, uint8_t& srcId);
uint8_t node.readBytes(uint8_t* outBuf, uint8_t* srcBuf, uint8_t maxLen);
bool node.peek(uint8_t& data, uint8_t& srcId);
void node.flush();
```

- `read()` / `peek()`는 항상 데이터 바이트와 발신자 노드 ID를 함께 반환합니다 — `readBytes()`의 `outBuf[i]`와 `srcBuf[i]`는 1:1로 대응되며, 큐에 쌓인 연속된 바이트라도 발신자가 다를 수 있기 때문입니다.
- 수신 데이터는 내부 FIFO(`SWP2P_FIFO_DEPTH`, 기본 16, 2의 거듭제곱이어야 함)에 쌓이며, 자신에게 보내진 패킷 또는 브로드캐스트 패킷만 FIFO에 들어옵니다.

### 상태 확인

```cpp
bool node.isSending();
bool node.isBusy();
bool node.isAckFailed();
```

- `isAckFailed()`는 가장 최근 `send()`/`sendBurst()`가 ACK 타임아웃으로 실패했는지 조회합니다. 조회하는 순간 플래그가 자동으로 클리어되므로, 결과를 확인하고 싶은 전송 직후에 바로 체크하세요.

### 버퍼 API (`SWP2PBuffer<CAP>`)

여러 바이트를 비트/바이트 단위로 모아서 한 번에 버스트 전송할 때 사용합니다.

```cpp
SWP2PBuffer<16> buf;

// 비트 하나씩 밀어넣기 (8개 모이면 바이트 1개 완성)
node.buff(buf, digitalRead(10));

// 바이트 단위로 밀어넣기
node.buff(buf, someByte, true);

// buffer 전체(len()만큼) 전송
node.sendBurst(destId, buf);

// buf[4]부터 8바이트만 전송
node.sendBurst(destId, buf, 4, 8);

// buffer 전체 비우기
node.buffFree(buf);

// buf[2] 위치의 값만 0으로 비우기 (count는 그대로, 뒤 데이터는 당겨지지 않음)
node.buffFree(buf, 2);
```

- `SWP2PBuffer<CAP>`의 `CAP`은 컴파일타임 상수이며, `CAP > SWP2P_MAX_BURST`이면 컴파일 에러가 발생합니다.
- `buffFree(buf, idx)`는 삭제가 아니라 해당 위치 값을 0으로 덮어쓰는 동작입니다. `count`(버퍼 길이)에는 영향을 주지 않으므로, 이후 `sendBurst(dest, buf)`를 호출하면 그 자리의 0x00도 실제 데이터로 함께 전송됩니다.

### `DataPreset` (데이터선 구성)

| Preset | 데이터선 | 폭 |
|---|---|---|
| `PRESET_W1_D4`~`PRESET_W1_D7`, `D10`, `A0` | 1개 | 1비트 |
| `PRESET_W1_D9` | 1개 (D9) | 1비트 (슬레이브 전용, CLK 마스터 D9와 충돌) |
| `PRESET_W2_D4_D5`, `PRESET_W2_D6_D7`, `PRESET_W2_A0_A1` | 2개 | 2비트 |
| `PRESET_W2_D9_D10` | 2개 (D9,D10) | 2비트 (슬레이브 전용, CLK 마스터 D9와 충돌) |
| `PRESET_W4_D4_D7` (기본 권장), `PRESET_W4_A0_A3` | 4개 | 4비트 |
| `PRESET_W4_D9_D12` | 4개 (D9~D12) | 4비트 (슬레이브 전용, CLK 마스터 D9와 충돌) |
| `PRESET_W8_A0_D7` | 8개 | 8비트 |

폭이 넓을수록 한 번의 클럭 엣지에 더 많은 비트를 실어 보내 전송 사이클 수는 줄지만, 실측상 ISR 처리 시간이 병목이 되어 최대 클럭 주파수는 오히려 낮아지는 경향이 있었습니다(내부 실측 기준: WIDTH=1에서 약 40kHz, WIDTH=8에서 약 29kHz). 용도에 맞게 트레이드오프를 고려해 선택하세요.

## 프로토콜 개요

1. **HEADER (슬롯 기반 송신권 결정 및 헤더 전송)**: BUSY가 유휴(idle)로 돌아오면 슬롯 카운터를 0으로 초기화합니다. 매 CLK 상승 에지마다 각 노드는 자기 슬롯(`_slotCounter == _nodeId`)에서 BUSY 핀을 확인하고, 여전히 HIGH인 경우에만 LOW로 점유한 뒤 `DEST(6) + SRC(6) + BURST(1)` 헤더를 push-pull로 출력합니다. 다른 노드는 BUSY LOW를 감지하고 수신 상태로 진입합니다.
2. **LEN (버스트일 때만)**: `SWP2P_MAX_BURST`에 필요한 최소 비트로 `len - 2`를 인코딩하고, 프리셋 폭의 청크 경계까지 0으로 패딩합니다(단일 바이트 전송에는 없습니다).
3. **DATA**: 1바이트(단일 전송) 또는 `len`바이트(버스트)를 순서대로 전송합니다.
4. **ACK**: 송신 측은 데이터선은 놓아주지만 BUSY는 계속 잡은 채, 목적지 노드가 ACK을 LOW로 구동하는지 제한된 클럭 엣지 수만큼 기다립니다. 제 시간에 ACK이 오면 전송 성공으로 처리되고, 못 받으면 타임아웃되어 다음 조회 시 `isAckFailed()`가 `true`를 반환합니다. 어느 쪽이든 대기가 끝나면 BUSY는 해제되어 다른 노드가 버스를 사용할 수 있습니다.

수신 측은 자신의 주소(또는 브로드캐스트) 패킷만 FIFO에 캡처하고 목적지 노드만 ACK을 구동하며, 그 외 패킷/노드는 라인 상태만 따라가며 무시합니다.

## 설계상 제약 / 주의사항

- 제어선(BUSY, ACK)은 open-drain으로 동작하므로 반드시 외부 풀업 저항이 필요합니다. 데이터선은 푸시풀로 구동되지만 유휴 상태 플로팅 방지를 위해 풀업 저항 구성을 권장합니다.
- 노드 ID는 0~62이며, 63(`SWP2P_BROADCAST`)은 브로드캐스트 전용입니다. packed 13비트 헤더는 W1/W2에서 기존 16비트 헤더보다 각각 3/1클럭을 줄이고, W4/W8에서는 청크 올림 때문에 클럭 수가 같습니다.
- `SWP2P_MAX_BURST`는 8비트 길이 API에 따라 최대 255입니다. LEN 비트 수는 이 설정에서 컴파일타임으로 계산됩니다. 버스트 최댓값을 키우면 LEN 비트와 TX 버퍼 RAM 사용량도 증가합니다. `CAP`이 `SWP2P_MAX_BURST`를 넘으면 컴파일 에러가 나며, 자동 분할하지 않습니다.
- 새 packed header/LEN 형식은 이전 버전과 선로 호환되지 않으므로 한 버스의 모든 노드를 같은 형식으로 업데이트해야 합니다.
- 클럭 주파수는 사용 환경(풀업 저항값, 배선, WIDTH 설정)에 따라 상한이 달라지므로 실측을 통해 조정하는 것을 권장합니다.

## 파일 구성

- `SWP2P.h` — `SWP2PBase`(공통 static 상태) + `SWP2P<DataPreset>` 템플릿 클래스(핀 구성별 컴파일타임 특수화) + `SWP2P_BIND_ISRS` 매크로
- `SWP2P.cpp` — static 멤버 정의, Timer1 설정/정지, 공용 FIFO push 로직
- `SWP2PBuffer.h` — 비트/바이트 단위로 데이터를 모으는 사용자 친화 버퍼 템플릿 클래스
