/*
 * Sensor Board - Multi-sensor: 6 IR obstacle sensors
 *
 * Reads 6 MH Infrared Obstacle Sensors (LM393) every 100ms,
 * packs the readings into a snapshot struct, frames it with CRC16,
 * and transmits via UART2 to the control board.
 *
 * IR sensor output (all sensors):
 *   LOW  (0) = obstacle detected
 *   HIGH (1) = path clear
 *
 * Green LED flash on each send cycle.
 * Red LED on when ANY sensor detects obstacle.
 * Debug output via UART0 (OpenSDA virtual COM) at 115200 baud.
 */

#include "MKL25Z4.h"
#include "pin_config_tx.h"
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

/* ---- Main ---- */

int main(void)
{
    uint32_t   send_count = 0;
    uint8_t    seq = 0;
    snapshot_t snap;

    /* Frame buffer: big enough for sensor frames */
    uint8_t frame_buf[FRAME_HEADER_SIZE + SNAPSHOT_SIZE + FRAME_CRC_SIZE];
    uint8_t payload[SNAPSHOT_SIZE];

    /* Core clock, SysTick fires every 1 ms */
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    pin_config_init();
    pin_config_tx_init();
    uart2_init();
    debug_uart_init();

    RGB_ALL_OFF();

    PRINTF("[SENSOR] Multi-sensor: 6 IR obstacle sensors.\r\n");

    while (1) {
        /* Sample all 6 IR sensors */
        snapshot_sample(&snap);

        /* Visual feedback: red LED = any obstacle detected */
        if (snapshot_any_obstacle(&snap)) {
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
            for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
                debug_putdec(snap.ir[i]);
                if (i < NUM_IR_SENSORS - 1)
                    debug_putchar(',');
            }
            PRINTF("\r\n");
        }

        /* Wait for next send cycle (100ms total period) */
        delay_ms(90);
    }
}
