# CLAUDE.md — Capstone Communication

AI assistant guide for this repository. Read this before making any changes.

---

## Project Overview

This is a **bare-metal embedded C project** for an FRDM-KL25Z robotics capstone. Two boards communicate over UART2 at 9600 baud:

- **Sensor Board** (`sensor_board/`): Reads 6 IR obstacle sensors every 100ms, packs readings into framed packets with CRC16, and transmits to the control board.
- **Control Board** (`control_board/`): Receives frames via ISR + ring buffer, validates CRC, unpacks sensor snapshots, monitors for timeout (500ms = safe mode), and handles emergency stop from both a local button and remote ESTOP frames.

---

## Repository Structure

```
Capstone_Communication/
├── ARCHITECTURE.md          # Architecture design and implementation steps
├── CLAUDE.md                # This file
├── common/                  # Header-only shared modules (included by both boards)
│   ├── pin_config.h         # Shared pins: RGB LED + UART2 (call pin_config_init() first)
│   ├── pin_config_tx.h      # Sensor board pins: 6x IR sensors on PTE3/2, PTB11/10/9/8
│   ├── pin_config_rx.h      # Control board pins: ESTOP button on PTB3 (active-low)
│   ├── ringbuf.h            # Lock-free ring buffer (128B, power-of-2, ISR-safe)
│   ├── protocol.h           # Frame pack/unpack, CRC16-CCITT, parser state machine
│   ├── sensor_sample.h      # snapshot_t struct, sample/pack/unpack helpers
│   ├── uart.h               # UART2 driver: init, putchar, getchar (ISR-driven RX)
│   └── debug_uart.h         # UART0 debug driver (115200 baud, OpenSDA virtual COM)
├── sensor_board/
│   └── main.c               # Sensor board application (~160 lines)
└── control_board/
    └── main.c               # Control board application (~280 lines)
```

All modules in `common/` are **header-only** (all functions `static inline`). There are no `.c` files — everything compiles from headers included into the two `main.c` files.

---

## Technology Stack

| Item | Detail |
|------|--------|
| Language | Bare-metal C (C99) |
| Target MCU | NXP MKL25Z128VLK4 (ARM Cortex-M0+) |
| Board | FRDM-KL25Z |
| SDK Header | `MKL25Z4.h` (MCUXpresso SDK) |
| IDE | MCUXpresso (Eclipse-based) |
| Build system | MCUXpresso project files (no Makefile or CMake) |
| Debug interface | OpenSDA J-Link via USB |
| No external dependencies | Only CMSIS + MCUXpresso device headers |

---

## Communication Protocol

### Frame Format

```
| SOF (2B)  | LEN (1B) | TYPE (1B) | SEQ (1B) | PAYLOAD (N B) | CRC16 (2B) |
| 0xAA 0x55 |          | 0x01/0x02 | 0–255    |               | hi    lo   |
```

- **SOF**: `0xAA 0x55` — sync bytes, parser resyncs on any mismatch
- **LEN**: payload byte count (max 255)
- **TYPE**: `0x01` = `FRAME_TYPE_SENSOR`, `0x02` = `FRAME_TYPE_ESTOP`
- **SEQ**: incrementing 0–255, wraps; control board tracks sequence errors
- **CRC16**: CRC16-CCITT (poly `0x1021`, init `0xFFFF`) computed over `LEN + TYPE + SEQ + PAYLOAD`

### Sensor Snapshot Payload (8 bytes, `SNAPSHOT_SIZE`)

```
[0..5] ir_obs[0..5]  — 0 = obstacle detected, 1 = path clear
[6..7] reserved      — 0x00
```

### ESTOP Payload (1 byte)

```
[0] = 1 → activate emergency stop
[0] = 0 → release emergency stop
```

---

## Hardware Pin Assignments

### Shared (both boards) — `common/pin_config.h`

| Signal | Pin | Notes |
|--------|-----|-------|
| RGB Red | PTB18 | Active-low — use `RGB_RED_ON/OFF()` macros |
| RGB Green | PTB19 | Active-low — use `RGB_GREEN_ON/OFF()` macros |
| RGB Blue | PTD1 | Active-low — use `RGB_BLUE_ON/OFF()` macros |
| UART2 TX | PTD3 | ALT3 mux, board-to-board comm |
| UART2 RX | PTD2 | ALT3 mux, board-to-board comm |

- **UART2**: 9600 baud, 8N1, bus clock = 10,485,760 Hz (FEI mode)
- **RGB LEDs are active-low**: writing LOW turns ON, HIGH turns OFF

### Sensor Board — `common/pin_config_tx.h`

| Sensor Index | Pin | Notes |
|---|---|---|
| 0 | PTE3 | IR obstacle sensor (LM393) |
| 1 | PTE2 | IR obstacle sensor |
| 2 | PTB11 | IR obstacle sensor |
| 3 | PTB10 | IR obstacle sensor |
| 4 | PTB9 | IR obstacle sensor |
| 5 | PTB8 | IR obstacle sensor |

- All are GPIO inputs, no ADC
- `0` = obstacle detected (active-low), `1` = clear

### Control Board — `common/pin_config_rx.h`

| Signal | Pin | Notes |
|--------|-----|-------|
| ESTOP button | PTB3 | Active-low, internal pull-up; press = LOW |

### Debug UART (both boards) — `common/debug_uart.h`

| Signal | Pin | Notes |
|--------|-----|-------|
| UART0 RX | PTA1 | ALT2, OpenSDA virtual COM |
| UART0 TX | PTA2 | ALT2, 115200 baud |

---

## Key Design Patterns

### 1. ISR-driven RX with Ring Buffer

The UART2 RX ISR (`UART2_IRQHandler`) runs on the control board and does exactly one thing: push the received byte into `rx_ring`. The main loop drains the ring buffer continuously. Never poll `UART->S1` in the main loop.

```c
/* ISR (fast path — minimal work) */
void UART2_IRQHandler(void) {
    if (status & UART_S1_RDRF_MASK) {
        uint8_t c = COMM_UART->D;
        if (!ring_push(&rx_ring, c)) rx_overflow_count++;
    }
}

/* Main loop (slow path — drain and parse) */
while (uart2_getchar(&c)) {
    parser_feed(&parser, c);
}
```

### 2. Non-blocking State Machine Parser

`parser_feed()` in `protocol.h` consumes one byte per call and returns `PARSE_INCOMPLETE`, `PARSE_OK`, or `PARSE_BAD_CRC`. It never blocks. Call it from the main loop, not from an ISR.

### 3. Header-only Static Inline Pattern

All modules are implemented as `static inline` functions in `.h` files. This means:
- No `.c` files to compile separately
- Each `main.c` is a self-contained compilation unit
- Sensor board includes `pin_config_tx.h`; control board includes `pin_config_rx.h`

### 4. Timeout Fail-safe (500ms)

If the control board goes 500ms without a valid frame, it enters **safe mode**: blue LED steady, motor outputs set to 0. This is safety-critical for the demo — do not increase this threshold without good reason.

### 5. Volatile for ISR-shared Variables

All variables shared between ISR and main loop must be `volatile`:
- `rx_ring.head` / `rx_ring.tail` (in `ringbuf_t`)
- `rx_overflow_count`, `hw_overrun_count`
- `ms_ticks` (SysTick counter)

---

## Initialization Order

Both `main.c` files follow this exact init sequence:

```c
SystemCoreClockUpdate();
SysTick_Config(SystemCoreClock / 1000u);  // 1ms tick

pin_config_init();           // shared: RGB LED + UART2 clock gating
pin_config_tx_init();        // or pin_config_rx_init() for control board
uart2_init();                // enables UART2 + RX interrupt
debug_uart_init();           // UART0 at 115200 baud
```

Do **not** reorder these — clock gating must happen before pin/UART configuration.

---

## LED State Machine

| LED Color | Meaning |
|-----------|---------|
| Green flash | Valid frame received (control) / frame sent (sensor) |
| Red steady | Any IR obstacle currently detected |
| Blue steady | Safe mode (timeout) OR emergency stop active |
| 3x green blink | Sensor board startup confirmation |
| 3x blue blink | Control board startup confirmation |

---

## Debug Output

Both boards use `PRINTF(...)` (mapped to `debug_putchar` via `debug_uart.h`) on UART0 at 115,200 baud. Connect to OpenSDA virtual COM port to monitor.

**Sensor board** logs every 50 frames (~5 seconds):
```
[SENSOR] frames=50 seq=50 ir=111111
```

**Control board** logs every 5 seconds:
```
[STATS] good=50 bad_crc=0 seq_err=0 ir=111111 overflow=0 hw_overrun=0 estop=0
```

**Key diagnostic counters** (visible in IDE Watch window):
- `rx_overflow_count` — ring buffer full, byte dropped (should be 0)
- `hw_overrun_count` — UART hardware overrun (should be 0)
- `bad_crc` — CRC mismatches (should be 0)
- `seq_errors` — lost or out-of-order frames (should be 0)

---

## Development Conventions

### Code Style

- C99, `uint8_t`/`uint16_t`/`uint32_t` for all register-touching code
- Macro naming: `PREFIX_NAME` (e.g., `RGB_RED_ON`, `FRAME_SOF0`)
- Function naming: `module_verb_noun()` (e.g., `snapshot_pack`, `ring_push`)
- All common module functions are `static inline` — no separate `.c` files
- No dynamic memory allocation (`malloc`/`free`) — stack only
- `volatile` is required for any variable written in an ISR

### Adding a New Sensor or Message Type

1. Add a new `FRAME_TYPE_*` constant in `protocol.h`
2. Define the payload struct/size in `sensor_sample.h` (or a new header)
3. Pack the payload in `sensor_board/main.c` using `frame_pack()`
4. Handle the new type in the `PARSE_OK` block in `control_board/main.c`
5. Update pin config headers if new GPIO pins are needed

### Changing Baud Rate

The baud rate divisor is computed from `COMM_BAUD_RATE` in `pin_config.h` and the bus clock in `uart.h`. If you change the baud rate, recalculate the BDH/BDL/C4 register values in `uart2_init()`.

### Adding a New Board-Specific Pin

1. Add the pin definition to `pin_config_tx.h` or `pin_config_rx.h`
2. Add clock gating in the appropriate `pin_config_*_init()` function
3. Set PCR mux, direction, and initial state in the same init function

---

## Testing & Validation

There is no unit test framework. Testing is done in hardware following the 5-step progression in `ARCHITECTURE.md`:

| Step | What to Validate | Pass Criteria |
|------|-----------------|---------------|
| 1 | Byte-level UART connectivity | "HELLO" appears on control board serial monitor |
| 2 | Ring buffer under load | All counters = 0 after 10 min; `match_count ≈ seconds × 10` |
| 3 | Frame + CRC | `bad_crc = 0`, `good_frames` incrementing |
| 4 | Real sensor snapshot | `ir_obs[]` values update when sensors are triggered |
| 5 | Timeout fail-safe | Unplugging comm wire stops the car within 500ms |

---

## What NOT to Do

- **Do not poll `UART->S1` in the main loop** — always use `uart2_getchar()` which drains the ring buffer
- **Do not call `parser_feed()` from an ISR** — it is designed for the main loop only
- **Do not increase the timeout threshold** beyond 500ms without safety review
- **Do not add malloc/free** — this is a no-RTOS bare-metal environment
- **Do not use `#include <stdio.h>` `printf()`** — use `PRINTF()` from `debug_uart.h`
- **Do not modify `rx_ring.head` outside the ISR** or `rx_ring.tail` outside the main loop
- **Do not share a single `pin_config_tx.h` and `pin_config_rx.h`** in the same build — each board's `main.c` includes only its own board-specific config
