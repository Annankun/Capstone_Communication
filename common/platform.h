/*
 * platform.h - Minimal register definitions for FRDM-KL25Z (MKL25Z4)
 *
 * Only covers what we actually use: SIM, UART2, PORT, GPIO, PIT, SysTick.
 * If your board is different, adjust base addresses and pin mux values.
 */
#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>

/* ── Cortex-M0+ SysTick ───────────────────────────────────── */
#define SYST_CSR   (*(volatile uint32_t *)0xE000E010)
#define SYST_RVR   (*(volatile uint32_t *)0xE000E014)
#define SYST_CVR   (*(volatile uint32_t *)0xE000E018)

/* ── SIM (System Integration Module) ──────────────────────── */
#define SIM_SCGC4  (*(volatile uint32_t *)0x40048034)
#define SIM_SCGC5  (*(volatile uint32_t *)0x40048038)
#define SIM_SCGC6  (*(volatile uint32_t *)0x4004803C)
#define SIM_SOPT2  (*(volatile uint32_t *)0x40048004)

#define SIM_SCGC4_UART2_MASK  (1u << 12)
#define SIM_SCGC5_PORTD_MASK  (1u << 12)
#define SIM_SCGC5_PORTB_MASK  (1u << 10)
#define SIM_SCGC6_PIT_MASK    (1u << 23)

/* ── PORT D (pin mux for UART2 & GPIO) ───────────────────── */
#define PORTD_PCR2 (*(volatile uint32_t *)0x4004C008)  /* PTD2 = UART2_RX */
#define PORTD_PCR3 (*(volatile uint32_t *)0x4004C00C)  /* PTD3 = UART2_TX */
#define PORTD_PCR5 (*(volatile uint32_t *)0x4004C014)  /* PTD5 = GPIO (spare) */

/* PORT B (LED on FRDM-KL25Z: PTB18=Red, PTB19=Green) */
#define PORTB_PCR18 (*(volatile uint32_t *)0x4004A048)
#define PORTB_PCR19 (*(volatile uint32_t *)0x4004A04C)

/* ── GPIO ─────────────────────────────────────────────────── */
#define GPIOB_PDOR (*(volatile uint32_t *)0x400FF040)
#define GPIOB_PSOR (*(volatile uint32_t *)0x400FF044)
#define GPIOB_PCOR (*(volatile uint32_t *)0x400FF048)
#define GPIOB_PTOR (*(volatile uint32_t *)0x400FF04C)
#define GPIOB_PDDR (*(volatile uint32_t *)0x400FF054)

#define GPIOD_PDOR (*(volatile uint32_t *)0x400FF0C0)
#define GPIOD_PSOR (*(volatile uint32_t *)0x400FF0C4)
#define GPIOD_PCOR (*(volatile uint32_t *)0x400FF0C8)
#define GPIOD_PTOR (*(volatile uint32_t *)0x400FF0CC)
#define GPIOD_PDDR (*(volatile uint32_t *)0x400FF0D4)

/* ── UART2 registers ──────────────────────────────────────── */
#define UART2_BDH  (*(volatile uint8_t *)0x4006C000)
#define UART2_BDL  (*(volatile uint8_t *)0x4006C001)
#define UART2_C1   (*(volatile uint8_t *)0x4006C002)
#define UART2_C2   (*(volatile uint8_t *)0x4006C003)
#define UART2_S1   (*(volatile uint8_t *)0x4006C004)
#define UART2_S2   (*(volatile uint8_t *)0x4006C005)
#define UART2_C3   (*(volatile uint8_t *)0x4006C006)
#define UART2_D    (*(volatile uint8_t *)0x4006C007)

#define UART_C2_TE_MASK   (1u << 3)
#define UART_C2_RE_MASK   (1u << 2)
#define UART_C2_RIE_MASK  (1u << 5)
#define UART_S1_TDRE_MASK (1u << 7)
#define UART_S1_RDRF_MASK (1u << 5)

/* ── PIT (Periodic Interrupt Timer) ───────────────────────── */
#define PIT_MCR    (*(volatile uint32_t *)0x40037000)
#define PIT_LDVAL0 (*(volatile uint32_t *)0x40037100)
#define PIT_CVAL0  (*(volatile uint32_t *)0x40037104)
#define PIT_TCTRL0 (*(volatile uint32_t *)0x40037108)
#define PIT_TFLG0  (*(volatile uint32_t *)0x4003710C)

/* ── NVIC (interrupt enable) ──────────────────────────────── */
#define NVIC_ISER  (*(volatile uint32_t *)0xE000E100)
#define NVIC_ICER  (*(volatile uint32_t *)0xE000E180)

/* IRQ numbers for KL25Z */
#define UART2_IRQn   14
#define PIT_IRQn     22

/* ── Helpers ──────────────────────────────────────────────── */
static inline void enable_irq(int n)  { NVIC_ISER = (1u << n); }
static inline void disable_irq(int n) { NVIC_ICER = (1u << n); }

/* Default bus clock assumed 24 MHz (FRDM-KL25Z default after reset).
 * Adjust if your board uses a different clock tree.                  */
#define BUS_CLOCK_HZ  24000000u

#endif /* PLATFORM_H */
