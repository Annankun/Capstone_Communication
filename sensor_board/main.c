/*
 * Sensor Board - Step 4: Real snapshot payload + Emergency Stop
 *
 * Reads the MH Infrared Obstacle Sensor (LM393) on PTB2 every 100ms,
 * packs the reading into a snapshot struct, frames it with CRC16,
 * and transmits via UART2 to the control board.
 *
 * Emergency stop button on PTB3 (active-low, polled with debounce):
 *   Press once  = emergency stop (halt all TX, send ESTOP frame)
 *   Press again = resume (send resume frame, continue normal TX)
 *   Blue LED on = emergency stop active
 *
 * IR sensor output:
 *   LOW  (0) = obstacle detected
 *   HIGH (1) = path clear
 *
 * Green LED flash on each send cycle.
 * Red LED on when obstacle detected (visual feedback on sensor board).
 * Debug output via UART0 (OpenSDA virtual COM) at 115200 baud.
 */

#include "MKL25Z4.h"
#include "pin_config.h"
#include "uart.h"
#include "debug_uart.h"
#include "ringbuf.h"
#include "protocol.h"
#include "sensor_sample.h"

/* ---- Globals needed by uart.h (TX-only, but extern symbols must exist) ---- */

ringbuf_t           rx_ring;
volatile uint32_t   rx_overflow_count;

/* ---- Simple delay using SysTick ---- */

static volatile uint32_t ms_ticks;

void SysTick_Handler(void)
{
    ms_ticks++;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = ms_ticks;
    while ((ms_ticks - start) < ms)
        ;
}

/* ---- Decimal print helper ---- */

static void debug_putdec(uint32_t n)
{
    char tmp[10];
    int  i = 0;

    if (n == 0) {
        debug_putchar('0');
        return;
    }
    while (n > 0) {
        tmp[i++] = '0' + (char)(n % 10);
        n /= 10;
    }
    while (i > 0) {
        debug_putchar(tmp[--i]);
    }
}

/* ---- Emergency stop helpers ---- */

#define ESTOP_DEBOUNCE_MS  50u   /* debounce window */
#define ESTOP_PAYLOAD_SIZE 1u    /* 1 byte: 0x01=stop, 0x00=resume */

static void send_estop_frame(uint8_t *frame_buf, uint8_t seq, uint8_t active)
{
    uint8_t payload[1] = { active };
    uint8_t frame_len = frame_pack(frame_buf, FRAME_TYPE_ESTOP, seq,
                                    payload, ESTOP_PAYLOAD_SIZE);
    for (uint8_t i = 0; i < frame_len; i++)
        uart2_putchar(frame_buf[i]);
}

/* ---- Main ---- */

int main(void)
{
    uint32_t   send_count = 0;
    uint8_t    seq = 0;
    snapshot_t snap;

    /* Frame buffer: big enough for sensor or estop frames */
    uint8_t frame_buf[FRAME_HEADER_SIZE + SNAPSHOT_SIZE + FRAME_CRC_SIZE];
    uint8_t payload[SNAPSHOT_SIZE];

    /* Emergency stop state */
    uint8_t  estop_active   = 0;    /* 0 = normal, 1 = emergency stop */
    uint8_t  btn_prev       = 1;    /* previous button state (1 = released) */
    uint32_t btn_debounce_t = 0;    /* timestamp of last edge for debounce */

    /* Core clock, SysTick fires every 1 ms */
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    pin_config_init();
    uart2_init();
    debug_uart_init();

    RGB_ALL_OFF();

    PRINTF("[SENSOR] Step 4: IR sensor + E-STOP button (PTB3).\r\n");

    while (1) {
        /* ---- Poll emergency stop button (PTB3, active-low) ---- */
        uint8_t btn_now = (uint8_t)ESTOP_BTN_READ();

        if (btn_now == 0 && btn_prev == 1 &&
            (ms_ticks - btn_debounce_t) >= ESTOP_DEBOUNCE_MS) {
            /* Falling edge detected (button pressed) */
            btn_debounce_t = ms_ticks;
            estop_active = !estop_active;

            /* Send ESTOP frame to control board */
            send_estop_frame(frame_buf, seq, estop_active);
            seq++;

            if (estop_active) {
                RGB_ALL_OFF();
                RGB_BLUE_ON();
                PRINTF("[SENSOR] *** EMERGENCY STOP ACTIVATED ***\r\n");
            } else {
                RGB_BLUE_OFF();
                PRINTF("[SENSOR] *** EMERGENCY STOP RELEASED ***\r\n");
            }
        }
        btn_prev = btn_now;

        /* ---- Normal operation (only when not in emergency stop) ---- */
        if (!estop_active) {
            /* Sample the IR sensor */
            snapshot_sample(&snap);

            /* Visual feedback: red LED = obstacle detected */
            if (snap.ir_obstacle == 0) {
                RGB_RED_ON();
            } else {
                RGB_RED_OFF();
            }

            /* Serialize and pack into frame */
            snapshot_pack(&snap, payload);
            uint8_t frame_len = frame_pack(frame_buf, FRAME_TYPE_SENSOR, seq,
                                            payload, SNAPSHOT_SIZE);

            /* Transmit frame byte by byte */
            for (uint8_t i = 0; i < frame_len; i++)
                uart2_putchar(frame_buf[i]);

            seq++;
            send_count++;

            /* Brief green flash to show "sent" */
            RGB_GREEN_ON();
            delay_ms(10);
            RGB_GREEN_OFF();

            /* Log every 50 frames (~5 seconds) */
            if ((send_count % 50) == 0) {
                PRINTF("[SENSOR] frames=");
                debug_putdec(send_count);
                PRINTF(" seq=");
                debug_putdec(seq);
                PRINTF(" ir=");
                debug_putdec(snap.ir_obstacle);
                PRINTF("\r\n");
            }

            /* Wait for next send cycle (100ms total period) */
            delay_ms(90);
        } else {
            /* In emergency stop: just poll button frequently */
            delay_ms(10);
        }
    }
}
