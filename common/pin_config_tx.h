#ifndef PIN_CONFIG_TX_H
#define PIN_CONFIG_TX_H

/*
 * TX (Sensor Board) Pin Configuration - FRDM-KL25Z
 *
 * 6x MH Infrared Obstacle Sensors (LM393, digital out).
 * Include pin_config.h first for shared definitions.
 *
 * Sensor index   Pin     Port/GPIO
 * -----------   -----   ----------
 *     0         PTE3    PORTE / GPIOE
 *     1         PTE2    PORTE / GPIOE
 *     2         PTB11   PORTB / GPIOB
 *     3         PTB10   PORTB / GPIOB
 *     4         PTB9    PORTB / GPIOB
 *     5         PTB8    PORTB / GPIOB
 *
 * All sensors: active-low output (0 = obstacle, 1 = clear).
 */

#include "pin_config.h"

/*
 * TX-specific pin init — call after pin_config_init().
 * Configures all 6 obstacle sensor pins as GPIO input with pull-up.
 */
static inline void pin_config_tx_init(void)
{
    /* PTE3, PTE2: GPIO input with pull-up */
    PORTE->PCR[3]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTE->PCR[2]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOE->PDDR   &= ~((1u << 3) | (1u << 2));

    /* PTB11, PTB10, PTB9, PTB8: GPIO input with pull-up */
    PORTB->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[9]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[8]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOB->PDDR   &= ~((1u << 11) | (1u << 10) | (1u << 9) | (1u << 8));
}

/*
 * Read all 6 obstacle sensors in one shot (polling).
 * Reads PDIR once per port to get a consistent snapshot.
 *
 * obs[0..5]: 0 = obstacle detected, 1 = clear.
 */
static inline void ir_obs_read_all(uint8_t obs[6])
{
    uint32_t pdir_e = GPIOE->PDIR;
    uint32_t pdir_b = GPIOB->PDIR;

    obs[0] = (uint8_t)((pdir_e >> 3)  & 1u);   /* PTE3  */
    obs[1] = (uint8_t)((pdir_e >> 2)  & 1u);   /* PTE2  */
    obs[2] = (uint8_t)((pdir_b >> 11) & 1u);   /* PTB11 */
    obs[3] = (uint8_t)((pdir_b >> 10) & 1u);   /* PTB10 */
    obs[4] = (uint8_t)((pdir_b >> 9)  & 1u);   /* PTB9  */
    obs[5] = (uint8_t)((pdir_b >> 8)  & 1u);   /* PTB8  */
}

#endif /* PIN_CONFIG_TX_H */
