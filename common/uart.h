#ifndef UART_H
#define UART_H

#include "MKL25Z4.h"
#include "pin_config.h"
#include "ringbuf.h"

/*
 * UART2 driver — blocking TX, ISR-driven RX via ring buffer.
 * Bus clock assumed ~10.49 MHz (default FEI mode on FRDM-KL25Z).
 */

#define BUS_CLOCK_HZ  10485760u

extern ringbuf_t         rx_ring;
extern volatile uint32_t rx_overflow_count;

static inline void uart2_init(void)
{
    uint16_t sbr;

    /* Disable TX/RX while configuring */
    COMM_UART->C2 = 0;

    sbr = (uint16_t)(BUS_CLOCK_HZ / (16u * COMM_BAUD_RATE));
    COMM_UART->BDH = (uint8_t)((sbr >> 8) & 0x1F);
    COMM_UART->BDL = (uint8_t)(sbr & 0xFF);

    /* 8-N-1, no parity */
    COMM_UART->C1 = 0;

    COMM_UART->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK;

    ring_init(&rx_ring);
    rx_overflow_count = 0;

    NVIC_SetPriority(COMM_UART_IRQn, 2);
    NVIC_ClearPendingIRQ(COMM_UART_IRQn);
    NVIC_EnableIRQ(COMM_UART_IRQn);
}

/* Blocking send one byte */
static inline void uart2_putchar(uint8_t c)
{
    while (!(COMM_UART->S1 & UART_S1_TDRE_MASK))
        ;
    COMM_UART->D = c;
}

/* Blocking send a string */
static inline void uart2_puts(const char *s)
{
    while (*s)
        uart2_putchar((uint8_t)*s++);
}

/* Non-blocking receive; returns 1 if byte available, 0 otherwise. */
static inline int uart2_getchar(uint8_t *out)
{
    return ring_pop(&rx_ring, out);
}

#endif /* UART_H */
