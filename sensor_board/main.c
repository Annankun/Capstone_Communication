/*
 * Sensor Board - Step 1: Byte-level connectivity
 *
 * Sends "HELLO\n" every ~100ms via UART2.
 * Flashes GREEN LED on each send cycle.
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

/* ---- Main ---- */

int main(void)
{
    /* Core clock = 20.97 MHz, SysTick fires every 1 ms */
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    pin_config_init();
    uart2_init();

    RGB_ALL_OFF();

    while (1) {
        /* Send message */
        uart2_puts("HELLO\n");

        /* Flash green LED briefly to show "sent" */
        RGB_GREEN_ON();
        delay_ms(30);
        RGB_GREEN_OFF();

        /* Wait ~100ms between sends */
        delay_ms(70);
    }
}
