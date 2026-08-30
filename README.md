# SWP2P (Software Peer-to-Peer)

A masterless peer-to-peer communication library for AVR (Arduino) nodes.
Unlike SPI/I2C/CAN, where slave-to-slave traffic has to be relayed through a master,
SWP2P lets nodes talk to each other directly, without any node acting as a bus master.

## Features

- **Masterless architecture**: One node may drive the CLK line, but that node does not act as a bus master. All nodes participate in send/receive as equals.
- **Configurable data width**: Choose 1/2/4/8 data lines (`DataPreset`). More lines means more bits per clock edge and higher throughput.
- **CLK/BUSY_N/ACK_N based protocol**: Only a minimal set of control lines (BUSY_N, ACK_N) is needed, supporting roughly 10 nodes on the bus by design.
- **Bit-level arbitration**: The 2-byte destination + source address is driven open-drain and read back on the falling edge for comparison; on a collision the losing node yields automatically (TX_LOST → auto-retry).
- **Single-byte and burst transfers**: `send()` uses the original fast path (no LEN field); `sendBurst()` adds a LEN field to transfer multiple bytes in one frame.
- **Interrupt-driven, non-blocking**: Uses INT0 (CLK), INT1 (BUSY_N), PCINT0 (ACK_N), and Timer1 (automatic CLK output) to minimize CPU overhead.
- **RX FIFO + user-friendly buffer API**: `SWP2PBuffer<CAP>` lets you accumulate data bit-by-bit or byte-by-byte, then send it whole or as a sub-range.

## Hardware Layout

| Signal | Pin (default) | Description |
|---|---|---|
| CLK | D2 (INT0) | Communication clock. Driven by one node (not a master) or an external clock source |
| BUSY_N | D3 (INT1) | Bus-busy indicator |
| ACK_N | D8 (PCINT0) | Receive acknowledgment |
| Data line(s) | D4–D7 / D9,D10 / A0–A3, etc., depending on preset | Selected via `DataPreset` |

Data lines are always driven open-drain (pull-up resistors required). For each `DataPreset`, `_driveDataChunk` / `_readDataChunk` / `_dataRelease` are specialized at compile time (`if constexpr`) and fully inlined.

## Installation

Copy `SWP2P.h`, `SWP2P.cpp`, and `SWP2PBuffer.h` into a `libraries/SWP2P/` folder in your sketchbook, then `#include "SWP2P.h"` from the Arduino IDE (or arduino-cli).

## Quick Start

```cpp
#include "SWP2P.h"

// Node using 1 data line (D4), node ID = 1
SWP2P<PRESET_W1_D4> node(1);

void setup() {
    // Whether this node drives CLK, and the clock frequency (Hz)
    node.begin(true, 1000UL);
}

void loop() {
    // Send 1 byte to node 2
    node.send(2, 0x42);

    // Check for received data
    if (node.available()) {
        uint8_t v = node.read();
        // ...
    }
}
```

## API

### Node creation / initialization

```cpp
SWP2P<DataPreset> node(uint8_t nodeId);
void node.begin(bool clkIsOutput, unsigned long clkFreq = 100000UL);
```

- `nodeId` is masked to 7 bits (0–126). `0x7F` (127) is reserved as the broadcast address.
- If `clkIsOutput = true`, this node automatically drives CLK using Timer1.

### Sending

```cpp
bool node.send(uint8_t destId, uint8_t data);
bool node.sendBurst(uint8_t destId, const uint8_t* buf, uint8_t len);
```

- `send()` is the single-byte fast path (no LEN field).
- `sendBurst()` automatically sets the burst flag (MSB of the destination byte) when `len > 1`, and transmits an additional LEN field (encoded as `len - 2`). `len` must be between 1 and `SWP2P_MAX_BURST` (default 16, can be increased if needed).
- Returns `false` if a transfer is already in progress (`isSending()`) or the bus is busy (`isBusy()`).
- Use `SWP2P<PRESET>::BROADCAST` as `destId` to broadcast to all nodes.

### Receiving

```cpp
bool node.available();
uint8_t node.read();
uint8_t node.readBytes(uint8_t* outBuf, uint8_t maxLen);
uint8_t node.peek();
void node.flush();
```

Received data is queued in an internal FIFO (`SWP2P_FIFO_DEPTH`, default 16, must be a power of 2). Only packets addressed to this node (or broadcast packets) are captured into the FIFO.

### Status checks

```cpp
bool node.isSending();
bool node.isBusy();
```

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
| `PRESET_W1_D4`–`PRESET_W1_D10`, `PRESET_W1_A0` | 1 | 1 bit |
| `PRESET_W2_D4_D5`, `PRESET_W2_D6_D7`, `PRESET_W2_D9_D10`, `PRESET_W2_A0_A1` | 2 | 2 bits |
| `PRESET_W4_D4_D7`, `PRESET_W4_A0_A3`, `PRESET_W4_D9_D12` | 4 | 4 bits |
| `PRESET_W8_A0_D7` | 8 | 8 bits |

A wider data width carries more bits per clock edge, reducing the number of transfer cycles — but in practice, ISR execution time becomes the bottleneck, so the maximum achievable clock frequency can actually be lower with wider widths (measured internally: roughly 40kHz at WIDTH=1 vs. roughly 29kHz at WIDTH=8). Consider this trade-off when choosing a preset for your use case.

## Protocol Overview

1. **ARB (Arbitration)**: On each rising CLK edge, 2 bytes — `dest` (MSB = burst flag) and `src` — are driven open-drain, and read back on the falling edge for comparison. If the bus value indicates another node is driving a higher-priority (more "0"-heavy) value, this node immediately yields, enters the `TX_LOST` state, and automatically retries once BUSY_N returns to idle.
2. **LEN (burst frames only)**: The value `len - 2` is encoded and transmitted (this step is skipped for single-byte transfers).
3. **DATA**: 1 byte (single transfer) or `len` bytes (burst) are transmitted in sequence.
4. **RELEASE/ACK**: The sender releases the data line(s) and finishes; the receiver confirms reception via ACK_N.

Receiving nodes only capture packets addressed to them (or broadcast packets) into the FIFO; other packets are tracked for line-state purposes only and otherwise ignored.

## Design Constraints / Notes

- Data lines must be open-drain with pull-up resistors — testing confirmed that communication is impossible without pull-ups.
- Node IDs are limited to 0–126; 127 (`SWP2P_BROADCAST`) is reserved for broadcast.
- Increasing `SWP2P_MAX_BURST` (default 16) allows burst transfers of up to 257 bytes in theory, but increases RAM usage accordingly.
- The maximum usable clock frequency depends on your environment (pull-up resistor values, wiring, WIDTH setting); measuring and tuning it empirically is recommended.

## File Overview

- `SWP2P.h` — `SWP2PBase` (shared static state) + `SWP2P<DataPreset>` template class (compile-time specialization per pin configuration)
- `SWP2P.cpp` — static member definitions, Timer1 setup, ISR (INT0/INT1/PCINT0) bridges
- `SWP2PBuffer.h` — user-friendly buffer template class for accumulating data bit-by-bit or byte-by-byte


***



# SWP2P (Software Peer-to-Peer)

AVR(아두이노) 기반 마스터 없는 대등(peer-to-peer) 노드 간 통신 라이브러리입니다.
SPI/I2C/CAN처럼 마스터를 거쳐 슬레이브 ↔ 슬레이브 통신을 중계하는 오버헤드 없이,
노드끼리 직접 통신하는 것을 목표로 합니다.

## 특징

- **마스터 없는 구조**: CLK을 출력하는 노드가 있을 수는 있지만, 그 노드가 버스의 주인(마스터) 역할을 하지 않습니다. 모든 노드가 대등하게 송수신에 참여합니다.
- **가변 데이터 폭**: 데이터선을 1/2/4/8개 중에서 선택할 수 있습니다 (`DataPreset`). 선 개수가 늘어날수록 사이클당 더 많은 비트를 실어 전송 속도를 높일 수 있습니다.
- **CLK/BUSY_N/ACK_N 3선 기반 프로토콜**: 최소한의 제어선(BUSY_N, ACK_N)만으로 다수 노드(설계 목표 약 10개) 연결을 지원합니다.
- **비트 단위 중재(arbitration)**: 목적지 주소(dest) + 발신자 주소(src) 2바이트를 open-drain 방식으로 실어 보내고, 되읽기(readback) 비교를 통해 충돌 시 자동으로 양보(TX_LOST → 재시도)합니다.
- **단일 바이트 / 버스트 전송 모두 지원**: `send()`는 기존 빠른 경로를 그대로 사용하고, `sendBurst()`는 길이(LEN) 필드를 추가로 실어 여러 바이트를 한 번에 전송합니다.
- **인터럽트 기반, 논블로킹**: INT0(CLK), INT1(BUSY_N), PCINT0(ACK_N)과 Timer1(CLK 자동 출력)을 활용해 CPU 부담을 최소화합니다.
- **수신 FIFO + 사용자 친화 버퍼 API**: `SWP2PBuffer<CAP>`로 비트/바이트 단위로 데이터를 모으고, 그대로 혹은 부분 범위만 잘라서 전송할 수 있습니다.

## 하드웨어 구성

| 신호 | 핀 (기본) | 설명 |
|---|---|---|
| CLK | D2 (INT0) | 통신 클럭. 한 노드가 출력(마스터 아님) 또는 외부 클럭 |
| BUSY_N | D3 (INT1) | 버스 점유 여부 |
| ACK_N | D8 (PCINT0) | 수신 확인 |
| 데이터선 | Preset에 따라 D4~D7 / D9,D10 / A0~A3 등 | `DataPreset`으로 선택 |

데이터선은 항상 open-drain(풀업 필요)으로 구동되며, 각 `DataPreset`에 맞춰 컴파일타임에 `_driveDataChunk` / `_readDataChunk` / `_dataRelease`가 특수화(`if constexpr`)되어 인라인 전개됩니다.

## 설치

`SWP2P.h`, `SWP2P.cpp`, `SWP2PBuffer.h` 세 파일을 스케치북의 `libraries/SWP2P/` 폴더에 넣고 아두이노 IDE(또는 arduino-cli)에서 `#include "SWP2P.h"`로 사용합니다.

## 빠른 시작

```cpp
#include "SWP2P.h"

// 데이터선 1개(D4)를 사용하는 노드, 노드 ID = 1
SWP2P<PRESET_W1_D4> node(1);

void setup() {
    // 이 노드가 CLK을 출력할지 여부, 클럭 주파수(Hz)
    node.begin(true, 1000UL);
}

void loop() {
    // 노드 2번으로 1바이트 전송
    node.send(2, 0x42);

    // 수신 확인
    if (node.available()) {
        uint8_t v = node.read();
        // ...
    }
}
```

## API

### 노드 생성 / 초기화

```cpp
SWP2P<DataPreset> node(uint8_t nodeId);
void node.begin(bool clkIsOutput, unsigned long clkFreq = 100000UL);
```

- `nodeId`는 7비트(0~126)로 마스킹되어 저장됩니다. `0x7F`(127)은 브로드캐스트 전용 주소로 예약되어 있습니다.
- `clkIsOutput = true`이면 Timer1을 이용해 이 노드가 CLK을 자동 출력합니다.

### 전송

```cpp
bool node.send(uint8_t destId, uint8_t data);
bool node.sendBurst(uint8_t destId, const uint8_t* buf, uint8_t len);
```

- `send()`는 1바이트 전용 빠른 경로(LEN 필드 없음)입니다.
- `sendBurst()`는 `len > 1`일 때 목적지 주소의 MSB에 burst 플래그가 자동으로 세팅되고, LEN 필드(`len-2` 인코딩)가 추가로 전송됩니다. `len`은 1~`SWP2P_MAX_BURST`(기본 16, 필요 시 늘릴 수 있음) 범위여야 합니다.
- 이미 전송 중이거나(`isSending()`) 버스가 사용 중이면(`isBusy()`) `false`를 반환합니다.
- `destId`로 `SWP2P<PRESET>::BROADCAST`를 사용하면 모든 노드에 브로드캐스트됩니다.

### 수신

```cpp
bool node.available();
uint8_t node.read();
uint8_t node.readBytes(uint8_t* outBuf, uint8_t maxLen);
uint8_t node.peek();
void node.flush();
```

수신 데이터는 내부 FIFO(`SWP2P_FIFO_DEPTH`, 기본 16, 2의 거듭제곱이어야 함)에 쌓이며, 자신에게 보내진 패킷 또는 브로드캐스트 패킷만 FIFO에 들어옵니다.

### 상태 확인

```cpp
bool node.isSending();
bool node.isBusy();
```

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
| `PRESET_W1_D4`~`PRESET_W1_D10`, `PRESET_W1_A0` | 1개 | 1비트 |
| `PRESET_W2_D4_D5`, `PRESET_W2_D6_D7`, `PRESET_W2_D9_D10`, `PRESET_W2_A0_A1` | 2개 | 2비트 |
| `PRESET_W4_D4_D7`, `PRESET_W4_A0_A3`, `PRESET_W4_D9_D12` | 4개 | 4비트 |
| `PRESET_W8_A0_D7` | 8개 | 8비트 |

폭이 넓을수록 한 번의 클럭 엣지에 더 많은 비트를 실어 보내 전송 사이클 수는 줄지만, 실측상 ISR 처리 시간이 병목이 되어 최대 클럭 주파수는 오히려 낮아지는 경향이 있었습니다(내부 실측 기준: WIDTH=1에서 약 40kHz, WIDTH=8에서 약 29kHz). 용도에 맞게 트레이드오프를 고려해 선택하세요.

## 프로토콜 개요

1. **ARB (중재)**: CLK 상승 에지마다 `dest`(1바이트, MSB=burst 플래그) + `src`(1바이트) 총 2바이트를 open-drain으로 구동하고, 하강 에지에 되읽어 비교합니다. 자신이 구동한 값보다 버스 값이 더 "0"에 가까우면(상대가 더 우선순위 높은 값을 구동 중이면) 즉시 양보하고 `TX_LOST` 상태로 전환, BUSY_N이 idle로 돌아오면 자동 재시도합니다.
2. **LEN (버스트일 때만)**: `len - 2` 값을 인코딩해 전송합니다(단일 바이트 전송에는 없는 단계).
3. **DATA**: 1바이트(단일 전송) 또는 `len`바이트(버스트)를 순서대로 전송합니다.
4. **RELEASE/ACK**: 송신 측은 데이터선을 놓고(release) 종료하며, 수신 측은 ACK_N을 통해 수신을 확인합니다.

수신 측은 자신의 주소(또는 브로드캐스트) 패킷만 FIFO에 캡처하고, 그 외 패킷은 라인 상태만 따라가며 무시합니다.

## 설계상 제약 / 주의사항

- 데이터선은 반드시 풀업 저항이 있는 open-drain 구성이어야 합니다 (풀업이 없으면 통신 자체가 불가능함을 실측으로 확인).
- 노드 ID는 0~126만 사용 가능하며, 127(`SWP2P_BROADCAST`)은 브로드캐스트 전용입니다.
- `SWP2P_MAX_BURST`(기본 16)를 늘리면 이론상 최대 257바이트까지 버스트 전송이 가능하지만, RAM 사용량이 함께 증가합니다.
- 클럭 주파수는 사용 환경(풀업 저항값, 배선, WIDTH 설정)에 따라 상한이 달라지므로 실측을 통해 조정하는 것을 권장합니다.

## 파일 구성

- `SWP2P.h` — `SWP2PBase`(공통 static 상태) + `SWP2P<DataPreset>` 템플릿 클래스(핀 구성별 컴파일타임 특수화)
- `SWP2P.cpp` — static 멤버 정의, Timer1 설정, ISR(INT0/INT1/PCINT0) 브릿지
- `SWP2PBuffer.h` — 비트/바이트 단위로 데이터를 모으는 사용자 친화 버퍼 템플릿 클래스
