#ifndef PIN_CONFIG_RX_H
#define PIN_CONFIG_RX_H

/*
 * RX (Control Board) Pin Configuration - FRDM-KL25Z
 *
 * Pins specific to the control / receiver board.
 * Include pin_config.h first for shared definitions.
 */

#include "pin_config.h"

/* ---- Emergency Stop Button (active-low, external pull-up or internal) ---- */
#define ESTOP_BTN_PORT  PORTB
#define ESTOP_BTN_GPIO  GPIOB
#define ESTOP_BTN_PIN   3       /* PTB3 - GPIO input, press = LOW */

/* ---- Emergency stop button read (returns 0 = pressed, 1 = released) ---- */
#define ESTOP_BTN_READ() (((ESTOP_BTN_GPIO->PDIR) >> ESTOP_BTN_PIN) & 1u)

/*
 * RX-specific pin init — call after pin_config_init().
 */
static inline void pin_config_rx_init(void)
{
    /* Emergency stop button: GPIO input with pull-up */
    ESTOP_BTN_PORT->PCR[ESTOP_BTN_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    ESTOP_BTN_GPIO->PDDR &= ~(1u << ESTOP_BTN_PIN);  /* input */
}

#endif /* PIN_CONFIG_RX_H */
