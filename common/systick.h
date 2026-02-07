/*
 * systick.h - Simple millisecond tick using Cortex-M0+ SysTick
 *
 * Call systick_init() once, then use delay_ms() and millis().
 * Core clock assumed 48 MHz (FRDM-KL25Z default).
 */
#ifndef SYSTICK_H
#define SYSTICK_H

#include "platform.h"

#define CORE_CLOCK_HZ  48000000u

/* Volatile tick counter - incremented by SysTick_Handler */
extern volatile uint32_t g_systick_ms;

static inline void systick_init(void)
{
    SYST_RVR = (CORE_CLOCK_HZ / 1000u) - 1u;   /* 1 ms period */
    SYST_CVR = 0;
    SYST_CSR = (1u << 2) | (1u << 1) | (1u << 0);
    /* bits: CLKSOURCE=processor, TICKINT=1, ENABLE=1 */
}

static inline uint32_t millis(void) { return g_systick_ms; }

static inline void delay_ms(uint32_t ms)
{
    uint32_t start = g_systick_ms;
    while ((g_systick_ms - start) < ms)
        ;
}

#endif /* SYSTICK_H */
