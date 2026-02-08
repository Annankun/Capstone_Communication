/*
 * Control Board - Step 2: Ring buffer (ISR-driven RX)
 *
 * UART2 RX interrupt pushes bytes into a ring buffer.
 * Main loop pops bytes, feeds them into the HELLO matcher,
 * and tracks byte/message counters to validate no data loss.
 *
 * Debug output via UART0 (OpenSDA virtual COM) at 115200 baud.
 */

#include "MKL25Z4.h"
#include "pin_config.h"
#include "uart.h"
#include "debug_uart.h"

/* ---- Global ring buffer and overflow counter (used by uart.h) ---- */

ringbuf_t           rx_ring;
volatile uint32_t   rx_overflow_count;

/* ---- UART2 RX ISR ---- */

void UART2_IRQHandler(void)
{
    uint8_t status = COMM_UART->S1;

    if (status & UART_S1_RDRF_MASK) {
        uint8_t c = COMM_UART->D;    /* read clears RDRF */
        if (!ring_push(&rx_ring, c)) {
            rx_overflow_count++;      /* ring full — byte lost */
        }
    }

    /* Clear overrun flag if set (must read S1 then D, already done above) */
    if (status & UART_S1_OR_MASK) {
        (void)COMM_UART->D;          /* clear OR flag */
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

/* ---- Message matcher ---- */

static const char expected[] = "HELLO\n";
#define MSG_LEN 6   /* strlen("HELLO\n") */

static uint8_t match_idx;

/*
 * Feed one byte into the matcher.
 * Returns:  1 = full match,  -1 = mismatch (reset),  0 = still matching
 */
static int match_byte(uint8_t c)
{
    if (c == (uint8_t)expected[match_idx]) {
        match_idx++;
        if (match_idx == MSG_LEN) {
            match_idx = 0;
            return 1;      /* full match */
        }
        return 0;          /* partial match */
    }

    /* Mismatch: reset */
    match_idx = 0;
    if (c == (uint8_t)expected[0]) {
        match_idx = 1;
    }
    return -1;
}

/* ---- LED flash helpers ---- */

static void flash_green(void)
{
    RGB_GREEN_ON();
    delay_ms(50);
    RGB_GREEN_OFF();
}

static void flash_red(void)
{
    RGB_RED_ON();
    delay_ms(50);
    RGB_RED_OFF();
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
    uint32_t rx_byte_count   = 0;
    uint32_t match_count     = 0;
    uint32_t mismatch_count  = 0;
    uint32_t last_report     = 0;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    pin_config_init();
    uart2_init();       /* enables RX ISR + ring buffer */
    debug_uart_init();

    RGB_ALL_OFF();
    match_idx = 0;

    PRINTF("[CONTROL] Step 2: ISR + ring buffer RX. Waiting.\r\n");

    while (1) {
        uint8_t c;

        /* Drain ring buffer */
        while (uart2_getchar(&c)) {
            rx_byte_count++;

            int result = match_byte(c);
            if (result == 1) {
                match_count++;
                flash_green();
            } else if (result == -1) {
                mismatch_count++;
                flash_red();
            }
        }

        /* Periodic status report every 5 seconds */
        if ((ms_ticks - last_report) >= 5000u) {
            last_report = ms_ticks;

            PRINTF("[STATS] rx_bytes=");
            debug_putdec(rx_byte_count);
            PRINTF(" matches=");
            debug_putdec(match_count);
            PRINTF(" mismatches=");
            debug_putdec(mismatch_count);
            PRINTF(" overflow=");
            debug_putdec(rx_overflow_count);
            PRINTF(" ring_pending=");
            debug_putdec(ring_count(&rx_ring));
            PRINTF("\r\n");
        }
    }
}
