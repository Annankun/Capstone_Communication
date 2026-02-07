/*
 * led.h - Minimal LED helpers for FRDM-KL25Z
 *
 * On-board LEDs (active LOW on FRDM-KL25Z):
 *   PTB18 = Red LED
 *   PTB19 = Green LED
 *
 * Adjust pin assignments if your custom board differs.
 */
#ifndef LED_H
#define LED_H

#include "platform.h"

#define LED_RED_PIN   18
#define LED_GREEN_PIN 19

static inline void led_init(void)
{
    /* Clock gate PORT B */
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    /* Pin mux = GPIO (ALT1) */
    PORTB_PCR18 = (1u << 8);
    PORTB_PCR19 = (1u << 8);

    /* Direction = output */
    GPIOB_PDDR |= (1u << LED_RED_PIN) | (1u << LED_GREEN_PIN);

    /* Start with both OFF (active low → set high) */
    GPIOB_PSOR = (1u << LED_RED_PIN) | (1u << LED_GREEN_PIN);
}

/* Toggle */
static inline void led_red_toggle(void)   { GPIOB_PTOR = (1u << LED_RED_PIN); }
static inline void led_green_toggle(void) { GPIOB_PTOR = (1u << LED_GREEN_PIN); }

/* Explicit on/off (active-low: clear=on, set=off) */
static inline void led_red_on(void)    { GPIOB_PCOR = (1u << LED_RED_PIN); }
static inline void led_red_off(void)   { GPIOB_PSOR = (1u << LED_RED_PIN); }
static inline void led_green_on(void)  { GPIOB_PCOR = (1u << LED_GREEN_PIN); }
static inline void led_green_off(void) { GPIOB_PSOR = (1u << LED_GREEN_PIN); }

#endif /* LED_H */
