/*
 * Control Board - Step 3: Frame + parser (dummy payload)
 *
 * UART2 RX interrupt pushes bytes into a ring buffer.
 * Main loop drains ring buffer immediately into a non-blocking parser
 * state machine that validates SOF, extracts payload, and checks CRC.
 *
 * Tracks:
 *   - good_frames:  valid frames with correct CRC
 *   - bad_crc:      frames with CRC mismatch
 *   - seq_errors:   out-of-order or lost sequence numbers
 *
 * Heartbeat: if no valid frame for 500ms, enters safe mode (blue LED).
 *
 * Debug output via UART0 (OpenSDA virtual COM) at 115200 baud.
 */

#include "MKL25Z4.h"
#include "pin_config.h"
#include "uart.h"
#include "debug_uart.h"
#include "ringbuf.h"
#include "protocol.h"

/* ---- Global ring buffer and overflow counter (used by uart.h) ---- */

ringbuf_t           rx_ring;
volatile uint32_t   rx_overflow_count;
volatile uint32_t   hw_overrun_count;

/* ---- UART2 RX ISR ---- */

void UART2_IRQHandler(void)
{
    uint8_t status = COMM_UART->S1;

    if (status & UART_S1_RDRF_MASK) {
        uint8_t c = COMM_UART->D;    /* read clears RDRF */
        if (!ring_push(&rx_ring, c)) {
            rx_overflow_count++;      /* ring full - byte lost */
        }
    }

    /* Clear overrun flag if set (must read S1 then D, already done above) */
    if (status & UART_S1_OR_MASK) {
        (void)COMM_UART->D;          /* clear OR flag */
        hw_overrun_count++;
    }
}

/* ---- Simple delay using SysTick ---- */

static volatile uint32_t ms_ticks;

void SysTick_Handler(void)
{
    ms_ticks++;
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
    uint32_t good_frames   = 0;
    uint32_t bad_crc       = 0;
    uint32_t seq_errors    = 0;
    uint32_t rx_byte_count = 0;
    uint32_t last_report   = 0;
    uint32_t last_valid_rx = 0;   /* ms_ticks when last good frame received */
    uint8_t  expected_seq  = 0;
    uint8_t  first_frame   = 1;   /* flag: haven't received any frame yet */
    uint8_t  in_safe_mode  = 0;

    parser_t parser;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    pin_config_init();
    uart2_init();       /* enables RX ISR + ring buffer */
    debug_uart_init();

    RGB_ALL_OFF();
    parser_init(&parser);

    PRINTF("[CONTROL] Step 3: Frame parser active. Waiting.\r\n");

    while (1) {
        uint8_t c;

        /* Drain ring buffer immediately into parser - no delays */
        while (uart2_getchar(&c)) {
            rx_byte_count++;

            int result = parser_feed(&parser, c);

            if (result == PARSE_OK) {
                good_frames++;
                last_valid_rx = ms_ticks;

                /* Sequence check */
                if (first_frame) {
                    expected_seq = parser.seq + 1;
                    first_frame = 0;
                } else {
                    if (parser.seq != expected_seq)
                        seq_errors++;
                    expected_seq = parser.seq + 1;
                }

                /* Exit safe mode if we were in it */
                if (in_safe_mode) {
                    in_safe_mode = 0;
                    RGB_BLUE_OFF();
                }

                /* Brief green flash for valid frame */
                RGB_GREEN_ON();
            } else if (result == PARSE_BAD_CRC) {
                bad_crc++;
                /* Brief red flash for bad CRC */
                RGB_RED_ON();
            }
        }

        /* Turn off LEDs after drain (non-blocking visual feedback) */
        RGB_GREEN_OFF();
        RGB_RED_OFF();

        /* Heartbeat: 500ms without valid frame = safe mode */
        if (!first_frame && !in_safe_mode &&
            (ms_ticks - last_valid_rx) >= 500u) {
            in_safe_mode = 1;
            RGB_BLUE_ON();
            PRINTF("[CONTROL] SAFE MODE: no valid frame for 500ms!\r\n");
        }

        /* Periodic status report every 5 seconds */
        if ((ms_ticks - last_report) >= 5000u) {
            last_report = ms_ticks;

            PRINTF("[STATS] good=");
            debug_putdec(good_frames);
            PRINTF(" bad_crc=");
            debug_putdec(bad_crc);
            PRINTF(" seq_err=");
            debug_putdec(seq_errors);
            PRINTF(" rx_bytes=");
            debug_putdec(rx_byte_count);
            PRINTF(" overflow=");
            debug_putdec(rx_overflow_count);
            PRINTF(" hw_overrun=");
            debug_putdec(hw_overrun_count);
            PRINTF("\r\n");
        }
    }
}
