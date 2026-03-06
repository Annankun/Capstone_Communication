# Capstone Communication - Minimal Architecture Design

## Goal

- **Sensor Board**: Periodically send a "sensor snapshot" packet
- **Control Board**: Reliably receive the latest snapshot; stop motors on timeout

## Module Overview (5 modules)

### Common (shared by both boards)

| Module     | Responsibility                                      |
|------------|-----------------------------------------------------|
| `ringbuf`  | UART TX/RX circular buffer (ISR push, main loop pop)|
| `protocol` | Frame format, pack/checksum, parser state machine   |

### Sensor Board

| Module          | Responsibility                                  |
|-----------------|------------------------------------------------ |
| `sensor_sample` | Read sensors → produce `snapshot` struct        |
| `sensor_tx`     | PIT timer → pack snapshot into frame → TX ring  |

### Control Board

| Module       | Responsibility                                          |
|--------------|---------------------------------------------------------|
| `control_rx` | Feed RX ring → parser → update `latest_snapshot` + time |

## Frame Format

```
| SOF (2B)   | LEN (1B) | TYPE (1B) | SEQ (1B) | PAYLOAD (N B) | CRC16 (2B) |
| 0xAA 0x55  |          | 0x01      | 0-255    |                |             |
```

- SOF: Sync bytes `0xAA 0x55`
- LEN: Payload length (max 255)
- TYPE: Message type (`0x01` = SENSOR_SNAPSHOT)
- SEQ: Incrementing sequence number (wrap at 255)
- CRC16: Over LEN + TYPE + SEQ + PAYLOAD

## Implementation Steps

### Step 1: Byte-level connectivity
- Sensor sends `"HELLO\n"` every 100ms
- Control receives and confirms (LED / PC print)
- **Validates**: wiring, UART init, baud rate

### Step 2: Ring buffer (RX)
- RX ISR pushes bytes into `rx_ring`
- Main loop pops and counts
- **Validates**: no byte loss under load
- **Verification**: run 10 min, then check in IDE Watch window:
  - `rx_overflow_count == 0` (ring buffer never full)
  - `hw_overrun_count == 0` (UART hardware never overrun)
  - `mismatch_count == 0` (no garbled bytes)
  - `match_count ≈ seconds × 10` (message rate correct)

### Step 3: Frame + parser (dummy payload)
- Sensor sends framed packets (4-byte dummy payload, SEQ incrementing)
- Control parses frames, counts `good_frames` / `bad_crc`
- **Validates**: framing and CRC work correctly

### Step 4: Real snapshot payload
- Sensor copies snapshot atomically then sends
- Control overwrites `latest_snapshot` on valid frame
- **Validates**: all fields update consistently, no partial updates

### Step 5: Timeout fail-safe
- If `now - last_rx_time > 200ms` → motor output = 0
- **Validates**: unplug comm wire → car stops

## File Structure

```
common/
  pin_config.h          ← shared pins (RGB LED, UART2)
  pin_config_rx.h       ← RX / control board pins (ESTOP button)
  ringbuf.h / ringbuf.c
  protocol.h / protocol.c

sensor_board/
  ir_sensor.h           ← IR obstacle sensor driver (init + read)
  sensor_sample.h / sensor_sample.c
  sensor_tx.h / sensor_tx.c
  main.c                ← includes ir_sensor.h

control_board/
  control_rx.h / control_rx.c
  motor_ctrl.h / motor_ctrl.c
  main.c                ← includes pin_config_rx.h
```

## Pin Assignments (FRDM-KL25Z)

### Onboard RGB LED (active-low)

| Signal | Port/Pin | Function  |
|--------|----------|-----------|
| Red    | PTB18    | GPIO      |
| Green  | PTB19    | GPIO      |
| Blue   | PTD1     | GPIO      |

### UART2 Serial (board-to-board) — shared

| Signal | Port/Pin | ALT Mux |
|--------|----------|---------|
| TX     | PTD3     | ALT3    |
| RX     | PTD2     | ALT3    |

- Baud rate: 9600
- Configuration defined in `common/pin_config.h`

### TX (Sensor Board) Pins — `sensor_board/ir_sensor.h`

| Signal | Port/Pin | Function       |
|--------|----------|----------------|
| IR VOUT| PTB2     | GPIO input     |
| VCC    | 3.3V     | Power          |
| GND    | GND      | Ground         |

- IR Obstacle Sensor (MH-sensor, LM393)
- Output: LOW = obstacle detected, HIGH = path clear
- Digital output via LM393 comparator (no ADC needed)

### RX (Control Board) Pins — `common/pin_config_rx.h`

| Signal | Port/Pin | Function       |
|--------|----------|----------------|
| ESTOP  | PTB3     | GPIO input     |

- Emergency stop button (active-low, internal pull-up)
- Press = LOW, Release = HIGH

## Key Principles

1. Only one message type (snapshot) until stable
2. RX always via ISR + ringbuf (never poll UART register in main loop)
3. Parser runs in main loop (not in ISR)
4. Control always uses latest snapshot (no queue of old data)
5. Timeout → fail-safe (must work for demo safety)
