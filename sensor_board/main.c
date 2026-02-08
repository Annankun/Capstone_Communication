/*
 * Sensor Board - Step 3: Frame + parser (dummy payload)
 *
 * Sends framed packets every ~100ms via UART2 with:
 *   - 4-byte dummy payload (0xDE, 0xAD, 0xBE, 0xEF)
 *   - Incrementing sequence number (0-255, wraps)
 *   - CRC16 for integrity checking
 *
 * Green LED flash on each send cycle.
 * Debug output via UART0 (OpenSDA virtual COM) at 115200 baud.
 *
 * Validates: control board can parse framed packets and verify CRC.
 */

#include "MKL25Z4.h"
#include "pin_config.h"
#include "uart.h"
#include "debug_uart.h"
#include "ringbuf.h"
#include "protocol.h"

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

/* ---- Send one raw byte via UART2 ---- */

static void uart2_sendbyte(uint8_t b)
{
    uart2_putchar(b);
}

/* ---- Main ---- */

int main(void)
{
    uint32_t send_count = 0;
    uint8_t  seq = 0;

    /* Dummy payload: fixed 4 bytes */
    static const uint8_t dummy_payload[4] = { 0xDE, 0xAD, 0xBE, 0xEF };

    /* Frame buffer: max = HEADER(5) + PAYLOAD(4) + CRC(2) = 11 bytes */
    uint8_t frame_buf[FRAME_HEADER_SIZE + 4 + FRAME_CRC_SIZE];

    /* Core clock = 20.97 MHz, SysTick fires every 1 ms */
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    pin_config_init();
    uart2_init();
    debug_uart_init();

    RGB_ALL_OFF();

    PRINTF("[SENSOR] Step 3: Sending framed packets every 100ms.\r\n");

    while (1) {
        /* Pack frame */
        uint8_t frame_len = frame_pack(frame_buf, FRAME_TYPE_SENSOR, seq,
                                        dummy_payload, sizeof(dummy_payload));

        /* Transmit frame byte by byte */
        for (uint8_t i = 0; i < frame_len; i++)
            uart2_sendbyte(frame_buf[i]);

        seq++;
        send_count++;

        /* Brief green flash to show "sent" */
        RGB_GREEN_ON();
        delay_ms(10);
        RGB_GREEN_OFF();

        /* Log every 50 frames (~5 seconds) */
        if ((send_count % 50) == 0) {
            PRINTF("[SENSOR] frames_sent=");
            debug_putdec(send_count);
            PRINTF(" seq=");
            debug_putdec(seq);
            PRINTF("\r\n");
        }

        /* Wait for next send cycle (100ms total period) */
        delay_ms(90);
    }
}
