#ifndef PIN_CONFIG_TX_H
#define PIN_CONFIG_TX_H

/*
 * TX (Sensor Board) Pin Configuration - FRDM-KL25Z
 *
 * Pins specific to the sensor / transmitter board.
 * Include pin_config.h first for shared definitions.
 */

#include "pin_config.h"

/* ---- IR Obstacle Sensor (MH-sensor, LM393, digital out) ---- */
#define IR_OBS_PORT     PORTB
#define IR_OBS_GPIO     GPIOB
#define IR_OBS_PIN      2       /* PTB2 - digital input from VOUT */

/* ---- IR sensor read (returns 0 = obstacle, 1 = clear) ----
 * Hardware note: MH-sensor LM393 module outputs LOW when obstacle
 * is detected (active-low), which already matches the convention
 * 0 = obstacle, 1 = clear — no inversion needed.
 */
#define IR_OBS_READ()  (((IR_OBS_GPIO->PDIR) >> IR_OBS_PIN) & 1u)

/*
 * TX-specific pin init — call after pin_config_init().
 */
static inline void pin_config_tx_init(void)
{
    /* IR obstacle sensor: GPIO input with pull-up */
    IR_OBS_PORT->PCR[IR_OBS_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IR_OBS_GPIO->PDDR &= ~(1u << IR_OBS_PIN);  /* input */
}

#endif /* PIN_CONFIG_TX_H */
