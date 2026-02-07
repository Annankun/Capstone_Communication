/*
 * Control Board - main.c
 *
 * Compile with -DPHASE=1 through -DPHASE=6.
 *
 *   Phase 1: RX interrupt counts bytes, toggle LED on each byte
 *   Phase 2: RX interrupt → ring buffer, main loop pops & counts
 *   Phase 3: Ring buffer → frame parser, stats (good/bad/sync)
 *   Phase 4: Parse snapshot_v1 from valid frames
 *   Phase 5: (no control-side change for PIT)
 *   Phase 6: Fail-safe: stop motors on communication timeout
 */

#include "../common/platform.h"
#include "../common/systick.h"
#include "../common/uart2.h"
#include "../common/led.h"

#if PHASE >= 2
#include "../common/ringbuf.h"
#endif

#if PHASE >= 3
#include "../common/protocol.h"
#endif

#if PHASE >= 4
#include "../sensor_board/sensor_sample.h"   /* for snapshot_v1_t */
#endif

#if PHASE >= 6
#include "motor_ctrl.h"
#endif

#include <string.h>

/* ── SysTick tick counter ─────────────────────────────────── */
volatile uint32_t g_systick_ms = 0;

void SysTick_Handler(void)
{
    g_systick_ms++;
}

/* ── Phase 1: Simple byte counter ─────────────────────────── */
#if PHASE == 1
static volatile uint32_t g_rx_count = 0;

void UART2_IRQHandler(void)
{
    if (UART2_S1 & UART_S1_RDRF_MASK) {
        (void)UART2_D;         /* read & discard to clear flag */
        g_rx_count++;
        led_green_toggle();    /* visual confirmation */
    }
}
#endif

/* ── Phase 2+: RX into ring buffer ────────────────────────── */
#if PHASE >= 2
static ringbuf_t g_rx_ring;

void UART2_IRQHandler(void)
{
    if (UART2_S1 & UART_S1_RDRF_MASK) {
        uint8_t byte = UART2_D;
        ringbuf_push(&g_rx_ring, byte);
    }
}
#endif

/* ── Communication state (Phase 4+) ──────────────────────── */
#if PHASE >= 4
static snapshot_v1_t g_latest_snapshot;
static volatile uint32_t g_last_rx_time = 0;
#endif

/* ── Phase 6: Comms state ─────────────────────────────────── */
#if PHASE >= 6
typedef enum {
    COMMS_OK,
    COMMS_LOST
} comms_state_t;

static comms_state_t g_comms_state = COMMS_LOST;

#define COMMS_TIMEOUT_MS  200u
#endif

/* ── Main ─────────────────────────────────────────────────── */
int main(void)
{
    /* --- Common init ----------------------------------- */
    systick_init();
    led_init();
    uart2_init(115200, 1);  /* RX interrupt enabled */

#if PHASE >= 2
    ringbuf_init(&g_rx_ring);
#endif

#if PHASE >= 3
    frame_parser_t parser;
    parser_init(&parser);
#endif

#if PHASE >= 6
    motor_init();
    motor_stop();
#endif

    /* Counters for debug / LED feedback */
#if PHASE == 2
    uint32_t pop_count = 0;
#endif

    /* --- Super-loop ------------------------------------ */
    for (;;) {

        /* ========== Phase 1: just watch the counter ==== */
#if PHASE == 1
        /*
         * LED toggles in ISR.  Main loop can do other work.
         * If you have a debug UART to PC, print g_rx_count here.
         *
         * Visual check: LED should blink at ~10 Hz (100ms TX rate).
         * Unplug TX wire → LED stops.
         */
        delay_ms(500);
        /* Toggle red LED every 500ms as "alive" heartbeat */
        led_red_toggle();
#endif

        /* ========== Phase 2: ring buffer pop & count === */
#if PHASE == 2
        {
            uint8_t byte;
            while (ringbuf_pop(&g_rx_ring, &byte) == 0) {
                pop_count++;
                /* Blink green LED every 256 bytes */
                if ((pop_count & 0xFF) == 0)
                    led_green_toggle();
            }
        }
#endif

        /* ========== Phase 3+: frame parser ============= */
#if PHASE >= 3
        {
            uint8_t byte;
            while (ringbuf_pop(&g_rx_ring, &byte) == 0) {
                int got_frame = parser_feed(&parser, byte);

                if (got_frame) {
                    led_green_toggle();   /* blink on valid frame */

#if PHASE >= 4
                    /* Copy snapshot from parser payload */
                    if (parser.type == MSG_TYPE_SNAPSHOT &&
                        parser.len == sizeof(snapshot_v1_t))
                    {
                        memcpy(&g_latest_snapshot,
                               parser.payload,
                               sizeof(snapshot_v1_t));
                        g_last_rx_time = millis();
                    }
#endif
                }
            }
        }
#endif

        /* ========== Phase 6: fail-safe check =========== */
#if PHASE >= 6
        {
            uint32_t now = millis();
            uint32_t gap = now - g_last_rx_time;

            if (gap > COMMS_TIMEOUT_MS) {
                /* Communication lost → stop motors */
                if (g_comms_state != COMMS_LOST) {
                    motor_stop();
                    g_comms_state = COMMS_LOST;
                    led_red_on();    /* red = comms lost */
                }
            } else {
                /* Communication OK */
                if (g_comms_state != COMMS_OK) {
                    g_comms_state = COMMS_OK;
                    led_red_off();
                }
                /*
                 * Here you use g_latest_snapshot for your control logic:
                 *   - line following
                 *   - obstacle avoidance
                 *   - encoder-based speed control
                 * Example:
                 *   motor_set(compute_left(g_latest_snapshot),
                 *             compute_right(g_latest_snapshot));
                 */
            }
        }
#endif

    }
    /* never reached */
    return 0;
}
