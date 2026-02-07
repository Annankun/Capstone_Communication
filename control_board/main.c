/*
 * Control Board - Step 1: Byte-level connectivity
 *
 * Receives bytes from UART2, matches against "HELLO\n".
 * On successful match  -> flash GREEN LED  (same as sender)
 * On mismatch / error  -> flash RED LED
 *
 * Validates: wiring, UART init, baud rate.
 */

#include "MKL25Z4.h"
#include "../common/pin_config.h"
#include "../common/uart.h"

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

static uint8_t buf[MSG_LEN];
static uint8_t idx;

/*
 * Feed one byte into the matcher.
 * Returns:  1 = full match,  -1 = mismatch (reset),  0 = still matching
 */
static int match_byte(uint8_t c)
{
    if (c == (uint8_t)expected[idx]) {
        buf[idx] = c;
        idx++;
        if (idx == MSG_LEN) {
            idx = 0;       /* reset for next message */
            return 1;      /* full match */
        }
        return 0;          /* partial match, keep going */
    }

    /* Mismatch: reset */
    idx = 0;

    /*
     * The mismatched byte might be the start of a new 'H'.
     * Re-check it against expected[0].
     */
    if (c == (uint8_t)expected[0]) {
        idx = 1;
    }

    return -1;              /* mismatch */
}

/* ---- LED flash helpers ---- */

static void flash_green(void)
{
    RGB_GREEN_ON();
    delay_ms(30);
    RGB_GREEN_OFF();
}

static void flash_red(void)
{
    RGB_RED_ON();
    delay_ms(30);
    RGB_RED_OFF();
}

/* ---- Main ---- */

int main(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    pin_config_init();
    uart2_init();

    RGB_ALL_OFF();
    idx = 0;

    while (1) {
        uint8_t c;
        if (uart2_getchar(&c)) {
            int result = match_byte(c);
            if (result == 1) {
                flash_green();    /* Match! Same color as sender */
            } else if (result == -1) {
                flash_red();      /* Mismatch: signal error */
            }
            /* result == 0: still accumulating, no LED action */
        }
    }
}
