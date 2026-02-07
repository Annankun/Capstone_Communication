/*
 * Sensor Board - main.c
 *
 * Compile with -DPHASE=1 through -DPHASE=6 to incrementally
 * build up functionality.  Each phase is additive.
 *
 *   Phase 1: Send 0xA5 every 100ms (physical link test)
 *   Phase 2: (no sensor-side change, same as Phase 1)
 *   Phase 3: Send framed packets with dummy payload
 *   Phase 4: Send framed packets with real snapshot struct
 *   Phase 5: Use PIT timer for fixed TX cadence
 *   Phase 6: (no sensor-side change, fail-safe is on Control)
 */

#include "../common/platform.h"
#include "../common/systick.h"
#include "../common/uart2.h"
#include "../common/led.h"

#if PHASE >= 3
#include "../common/protocol.h"
#endif

#if PHASE >= 4
#include "sensor_sample.h"
#endif

#include <string.h>

/* ── SysTick tick counter ─────────────────────────────────── */
volatile uint32_t g_systick_ms = 0;

void SysTick_Handler(void)
{
    g_systick_ms++;
}

/* ── Phase 5: PIT flag ────────────────────────────────────── */
#if PHASE >= 5
static volatile uint8_t g_tx_tick = 0;

/*
 * PIT channel 0 ISR - fires at the configured TX rate.
 * Just sets a flag; actual work is done in the main loop.
 */
void PIT_IRQHandler(void)
{
    PIT_TFLG0 = 1;   /* clear interrupt flag (w1c) */
    g_tx_tick = 1;
}

static void pit_init_ms(uint32_t period_ms)
{
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;    /* clock gate */
    PIT_MCR = 0;                           /* enable PIT module */

    /* Load value: (bus_clock * period_ms / 1000) - 1 */
    uint32_t ldval = (BUS_CLOCK_HZ / 1000u) * period_ms - 1u;
    PIT_LDVAL0 = ldval;

    PIT_TCTRL0 = (1u << 1) | (1u << 0);  /* TIE + TEN */
    enable_irq(PIT_IRQn);
}
#endif /* PHASE >= 5 */

/* ── Main ─────────────────────────────────────────────────── */
int main(void)
{
    /* --- Common init ----------------------------------- */
    systick_init();
    led_init();
    uart2_init(115200, 0);  /* TX only, no RX interrupt needed */

#if PHASE >= 3
    uint8_t tx_seq = 0;
    uint8_t frame_buf[FRAME_MAX_PAYLOAD + 7];
#endif

#if PHASE >= 4
    snapshot_v1_t snap;
    memset(&snap, 0, sizeof(snap));
#endif

#if PHASE >= 5
    /*
     * TX period: 20ms = 50 Hz.
     * Adjust to 10 for 100 Hz if your bus can handle it.
     */
    pit_init_ms(20);
#endif

    /* --- Super-loop ------------------------------------ */
    for (;;) {

        /* ========== Phase 1 & 2: send raw byte ========= */
#if PHASE <= 2
        delay_ms(100);
        uart2_putchar(0xA5);
        led_green_toggle();   /* blink on every send */
#endif

        /* ========== Phase 3: framed dummy payload ====== */
#if PHASE == 3
        delay_ms(50);

        uint8_t dummy[4];
        dummy[0] = (uint8_t)(tx_seq);
        dummy[1] = (uint8_t)(tx_seq + 1);
        dummy[2] = (uint8_t)(tx_seq + 2);
        dummy[3] = (uint8_t)(tx_seq + 3);

        uint32_t flen = frame_pack(frame_buf, MSG_TYPE_SNAPSHOT,
                                   tx_seq, dummy, sizeof(dummy));
        uart2_send(frame_buf, flen);
        tx_seq++;
        led_green_toggle();
#endif

        /* ========== Phase 4: real snapshot ============= */
#if PHASE == 4
        delay_ms(50);

        /* Atomic snapshot: read all sensors into local copy */
        sensor_sample(&snap);
        snap.seq = tx_seq;
        snap.timestamp_ms = millis();

        uint32_t flen = frame_pack(frame_buf, MSG_TYPE_SNAPSHOT,
                                   tx_seq,
                                   (const uint8_t *)&snap,
                                   sizeof(snap));
        uart2_send(frame_buf, flen);
        tx_seq++;
        led_green_toggle();
#endif

        /* ========== Phase 5+: PIT-driven send ========== */
#if PHASE >= 5
        if (g_tx_tick) {
            g_tx_tick = 0;

            sensor_sample(&snap);
            snap.seq = tx_seq;
            snap.timestamp_ms = millis();

            uint32_t flen = frame_pack(frame_buf, MSG_TYPE_SNAPSHOT,
                                       tx_seq,
                                       (const uint8_t *)&snap,
                                       sizeof(snap));
            uart2_send(frame_buf, flen);
            tx_seq++;
            led_green_toggle();
        }
#endif
    }
    /* never reached */
    return 0;
}
