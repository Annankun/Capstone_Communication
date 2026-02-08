#ifndef UART_H
#define UART_H

#include "MKL25Z4.h"
#include "pin_config.h"
#include "ringbuf.h"

/*
 * UART2 driver - Step 2: ISR-driven RX with ring buffer.
 *
 * TX: blocking (same as Step 1 - sufficient for our throughput).
 * RX: ISR pushes every received byte into rx_ring; main loop pops.
 *
 * Bus clock assumed ~20.97 MHz (default FEI mode on FRDM-KL25Z).
 */

#define BUS_CLOCK_HZ  10485760u   /* Default FEI bus clock */

/* RX ring buffer - shared between ISR and main loop */
extern ringbuf_t  rx_ring;

/* Overflow counter - incremented by ISR when ring is full */
extern volatile uint32_t rx_overflow_count;

static inline void uart2_init(void)
{
    uint16_t sbr;

    /* Clock gating already enabled by pin_config_init() */

    /* Disable TX/RX while configuring */
    COMM_UART->C2 = 0;

    /* Baud rate: SBR = BusClock / (16 * baud) */
    sbr = (uint16_t)(BUS_CLOCK_HZ / (16u * COMM_BAUD_RATE));
    COMM_UART->BDH = (uint8_t)((sbr >> 8) & 0x1F);
    COMM_UART->BDL = (uint8_t)(sbr & 0xFF);

    /* 8-N-1, no parity */
    COMM_UART->C1 = 0;

    /* Enable transmitter, receiver, and RX interrupt */
    COMM_UART->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK;

    /* Initialize RX ring buffer */
    ring_init(&rx_ring);
    rx_overflow_count = 0;

    /* Enable UART2 interrupt in NVIC */
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

/*
 * Non-blocking receive from ring buffer.
 * Returns 1 if byte available, 0 otherwise.
 */
static inline int uart2_getchar(uint8_t *out)
{
    return ring_pop(&rx_ring, out);
}

#endif /* UART_H */
