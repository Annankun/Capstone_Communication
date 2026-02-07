/*
 * uart2.h - UART2 init / TX helpers (bare-metal, KL25Z)
 *
 * TX side: blocking poll (fine for our low-rate sends).
 * RX side: handled via ISR in each board's main.c or control_rx.c.
 */
#ifndef UART2_H
#define UART2_H

#include "platform.h"

/*
 * uart2_init - set up UART2 on PTD3(TX) / PTD2(RX), 8N1
 *
 * @baud: desired baud rate (e.g. 115200)
 * @rx_irq: if non-zero, enable RX interrupt (RDRF → NVIC)
 */
static inline void uart2_init(uint32_t baud, int rx_irq)
{
    /* 1. Clock gate: UART2 + PORTD */
    SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    /* 2. Pin mux: PTD3 = ALT3 (UART2_TX), PTD2 = ALT3 (UART2_RX) */
    PORTD_PCR3 = (3u << 8);   /* MUX = ALT3 */
    PORTD_PCR2 = (3u << 8);   /* MUX = ALT3 */

    /* 3. Disable TX/RX while configuring */
    UART2_C2 = 0;

    /* 4. Baud rate: SBR = BUS_CLOCK / (16 * baud) */
    uint16_t sbr = (uint16_t)(BUS_CLOCK_HZ / (16u * baud));
    UART2_BDH = (uint8_t)((sbr >> 8) & 0x1F);
    UART2_BDL = (uint8_t)(sbr & 0xFF);

    /* 5. 8N1, no parity (defaults are fine, just clear C1) */
    UART2_C1 = 0;

    /* 6. Enable TX and RX; optionally enable RX interrupt */
    uint8_t c2 = UART_C2_TE_MASK | UART_C2_RE_MASK;
    if (rx_irq)
        c2 |= UART_C2_RIE_MASK;
    UART2_C2 = c2;

    /* 7. Enable UART2 IRQ in NVIC if requested */
    if (rx_irq)
        enable_irq(UART2_IRQn);
}

/* Blocking single-byte transmit */
static inline void uart2_putchar(uint8_t c)
{
    while (!(UART2_S1 & UART_S1_TDRE_MASK))
        ;
    UART2_D = c;
}

/* Blocking send N bytes */
static inline void uart2_send(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
        uart2_putchar(buf[i]);
}

#endif /* UART2_H */
