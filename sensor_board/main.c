/*
 * Sensor Board - Step 1: Byte-level connectivity
 *
 * Sends "HELLO\n" every ~1s via UART2.
 * Flashes GREEN LED on each send cycle.
 * Debug output via UART0 (OpenSDA virtual COM) at 115200 baud.
 *
 * Validates: wiring, UART init, baud rate.
 */

#include "MKL25Z4.h"
#include "../common/pin_config.h"
#include "../common/uart.h"
#include "../common/debug_uart.h"

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
    debug_uart_init();

    RGB_ALL_OFF();

    PRINTF("[SENSOR] Booted. Sending HELLO every 1s.\r\n");

    while (1) {
        /* Send message */
        uart2_puts("HELLO\n");

        PRINTF("[SENSOR] Sent: HELLO\\n\r\n");

        /* Flash green LED to show "sent" */
        RGB_GREEN_ON();
        delay_ms(500);
        RGB_GREEN_OFF();

        /* Wait between sends */
        delay_ms(500);
    }
}
