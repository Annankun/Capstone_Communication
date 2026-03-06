#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "MKL25Z4.h"

/*
 * Debug UART0 — outputs to OpenSDA virtual COM port on FRDM-KL25Z.
 * Pins: PTA1 = UART0_RX (ALT2), PTA2 = UART0_TX (ALT2)
 * Baud: 115200
 */

#define DEBUG_BAUD_RATE  115200u
#define DEBUG_CLOCK_HZ   20971520u   /* MCGFLLCLK in FEI mode */

static inline void debug_uart_init(void)
{
    uint16_t sbr;

    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);   /* MCGFLLCLK as UART0 clock */

    PORTA->PCR[1] = PORT_PCR_MUX(2);   /* UART0_RX */
    PORTA->PCR[2] = PORT_PCR_MUX(2);   /* UART0_TX */

    UART0->C2 = 0;

    /*
     * OSR=13 gives SBR=14 → actual baud 115228 Hz (0.024% error).
     * Default OSR=16 gives SBR=11 → 3.4% error (too high for reliable comms).
     */
    #define DEBUG_OSR  13u
    sbr = (uint16_t)(DEBUG_CLOCK_HZ / (DEBUG_OSR * DEBUG_BAUD_RATE));
    UART0->BDH = (uint8_t)((sbr >> 8) & 0x1F);
    UART0->BDL = (uint8_t)(sbr & 0xFF);
    UART0->C4  = (UART0->C4 & ~0x1Fu) | (uint8_t)(DEBUG_OSR - 1u);

    UART0->C1 = 0;
    UART0->C2 = UART0_C2_TE_MASK | UART0_C2_RE_MASK;
}

/* Blocking send one byte */
static inline void debug_putchar(char c)
{
    while (!(UART0->S1 & UART0_S1_TDRE_MASK))
        ;
    UART0->D = (uint8_t)c;
}

/* Blocking send a string */
static inline void debug_puts(const char *s)
{
    while (*s)
        debug_putchar(*s++);
}

/* Print one byte as two hex digits */
static inline void debug_puthex(uint8_t b)
{
    static const char hex[] = "0123456789ABCDEF";
    debug_putchar(hex[b >> 4]);
    debug_putchar(hex[b & 0x0F]);
}

/* Print byte as char if printable, else as \xHH */
static inline void debug_print_byte(uint8_t c)
{
    if (c >= 0x20 && c <= 0x7E) {
        debug_putchar((char)c);
    } else if (c == '\n') {
        debug_puts("\\n");
    } else if (c == '\r') {
        debug_puts("\\r");
    } else {
        debug_puts("\\x");
        debug_puthex(c);
    }
}

#define PRINTF  debug_puts

#endif /* DEBUG_UART_H */
