/*
 * Sensor Board - Step 2: Higher-rate TX for ring buffer stress test
 *
 * Sends "HELLO\n" every ~100ms via UART2 (10x faster than Step 1).
 * Flashes GREEN LED briefly on each send cycle.
 * Debug output via UART0 (OpenSDA virtual COM) at 115200 baud.
 *
 * Validates: control board ISR + ring buffer handles sustained load.
 */

#include "MKL25Z4.h"
#include "../common/pin_config.h"
#include "../common/uart.h"
#include "../common/debug_uart.h"

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
    uint32_t send_count = 0;

    /* Core clock = 20.97 MHz, SysTick fires every 1 ms */
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    pin_config_init();
    uart2_init();
    debug_uart_init();

    RGB_ALL_OFF();

    PRINTF("[SENSOR] Step 2: Sending HELLO every 100ms.\r\n");

    while (1) {
        /* Send message */
        uart2_puts("HELLO\n");
        send_count++;

        /* Brief green flash to show "sent" */
        RGB_GREEN_ON();
        delay_ms(10);
        RGB_GREEN_OFF();

        /* Log every 50 messages (~5 seconds) */
        if ((send_count % 50) == 0) {
            PRINTF("[SENSOR] sent=");
            debug_putdec(send_count);
            PRINTF("\r\n");
        }

        /* Wait for next send cycle (100ms total period) */
        delay_ms(90);
    }
}
