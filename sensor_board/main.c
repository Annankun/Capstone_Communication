/*
 * Sensor Board - Read sensor_status and transmit every 100ms
 *
 * Reads g_sensor_status (maintained by classmates) every 100ms,
 * packs the full status struct into a framed packet with CRC16,
 * and transmits via UART2 to the control board.
 *
 * Sensor status payload (22 bytes):
 *   ir[6]           0 = obstacle, 1 = clear
 *   ultrasonic[4]   0 = danger,   1 = safe
 *   tof             0 = danger,   1 = safe
 *   gps_valid       0 = no fix,   1 = fix
 *   gps_lat         int32_t latitude
 *   gps_lon         int32_t longitude
 *   padding[2]
 *
 * Green LED flash on each send cycle.
 * Red LED on when any obstacle/danger detected (visual feedback).
 * Debug output via UART0 (OpenSDA virtual COM) at 115200 baud.
 */

#include "MKL25Z4.h"
#include "pin_config_tx.h"
#include "uart.h"
#include "debug_uart.h"
#include "ringbuf.h"
#include "protocol.h"
#include "sensor_status.h"

/* ---- Global sensor status (classmates write, we read) ---- */

sensor_status_t g_sensor_status;

/* ---- Globals needed by uart.h (TX-only, but extern symbols must exist) ---- */

ringbuf_t           rx_ring;
volatile uint32_t   rx_overflow_count;

/* ---- UART2 RX ISR (must exist even on TX board: uart2_init enables RIE) ---- */

void UART2_IRQHandler(void)
{
    uint8_t status = COMM_UART->S1;

    /* Read data register to clear RDRF (and OR if set) — just discard */
    if (status & (UART_S1_RDRF_MASK | UART_S1_OR_MASK)) {
        (void)COMM_UART->D;
    }
}

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

    /* Frame buffer: header + sensor status payload + CRC */
    uint8_t frame_buf[FRAME_HEADER_SIZE + SENSOR_STATUS_SIZE + FRAME_CRC_SIZE];
    uint8_t payload[SENSOR_STATUS_SIZE];

    /* Core clock, SysTick fires every 1 ms */
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    pin_config_init();
    pin_config_tx_init();
    uart2_init();
    debug_uart_init();

    RGB_ALL_OFF();

    /* Startup blink: 3x green to confirm board is alive */
    for (int blink = 0; blink < 3; blink++) {
        RGB_GREEN_ON();
        delay_ms(150);
        RGB_GREEN_OFF();
        delay_ms(150);
    }

    PRINTF("[SENSOR] sensor_status TX: 22-byte payload every 100ms\r\n");
    PRINTF("[SENSOR] UART2 TX on PTD3, RX on PTD2, 9600 baud\r\n");

    while (1) {
        /* Read latest sensor status (classmates maintain g_sensor_status) */
        sensor_status_t snap = g_sensor_status;

        /* Visual feedback: red LED = any obstacle or danger */
        if (sensor_status_any_obstacle(&snap)) {
            RGB_RED_ON();
        } else {
            RGB_RED_OFF();
        }

        /* Serialize and pack into frame */
        sensor_status_pack(&snap, payload);
        uint8_t frame_len = frame_pack(frame_buf, FRAME_TYPE_SENSOR, seq,
                                        payload, SENSOR_STATUS_SIZE);

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
            for (uint8_t i = 0; i < 6u; i++)
                debug_putchar(snap.ir[i] ? '1' : '0');
            PRINTF(" us=");
            for (uint8_t i = 0; i < 4u; i++)
                debug_putchar(snap.ultrasonic[i] ? '1' : '0');
            PRINTF(" tof=");
            debug_putchar(snap.tof ? '1' : '0');
            PRINTF(" gps=");
            debug_putchar(snap.gps_valid ? '1' : '0');
            PRINTF("\r\n");
        }

        /* Wait for next send cycle (100ms total period) */
        delay_ms(90);
    }
}
