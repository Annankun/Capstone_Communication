#ifndef PIN_CONFIG_TX_H
#define PIN_CONFIG_TX_H

/*
 * TX (Sensor Board) Pin Configuration - FRDM-KL25Z
 *
 * Pins specific to the sensor / transmitter board.
 * Include pin_config.h first for shared definitions.
 *
 * 6 x MH Infrared Obstacle Sensors (LM393, digital out)
 *   Sensor 0: PTB2  (GPIOB)
 *   Sensor 1: PTC0  (GPIOC)
 *   Sensor 2: PTC3  (GPIOC)
 *   Sensor 3: PTC4  (GPIOC)
 *   Sensor 4: PTC5  (GPIOC)
 *   Sensor 5: PTC6  (GPIOC)
 */

#include "pin_config.h"

/* ---- Number of IR obstacle sensors ---- */
#define NUM_IR_SENSORS  6u

/* ---- Per-sensor port/GPIO/pin tables ---- */

/* GPIO port base pointers, indexed by sensor number */
#define IR_OBS_GPIO_0   GPIOB
#define IR_OBS_GPIO_1   GPIOC
#define IR_OBS_GPIO_2   GPIOC
#define IR_OBS_GPIO_3   GPIOC
#define IR_OBS_GPIO_4   GPIOC
#define IR_OBS_GPIO_5   GPIOC

/* PORT base pointers (for PCR configuration) */
#define IR_OBS_PORT_0   PORTB
#define IR_OBS_PORT_1   PORTC
#define IR_OBS_PORT_2   PORTC
#define IR_OBS_PORT_3   PORTC
#define IR_OBS_PORT_4   PORTC
#define IR_OBS_PORT_5   PORTC

/* Pin numbers within the port */
#define IR_OBS_PIN_0    2       /* PTB2 */
#define IR_OBS_PIN_1    0       /* PTC0 */
#define IR_OBS_PIN_2    3       /* PTC3 */
#define IR_OBS_PIN_3    4       /* PTC4 */
#define IR_OBS_PIN_4    5       /* PTC5 */
#define IR_OBS_PIN_5    6       /* PTC6 */

/* ---- Per-sensor read macros (returns 0 = obstacle, 1 = clear) ----
 * Hardware note: MH-sensor LM393 module outputs LOW when obstacle
 * is detected (active-low), which already matches the convention
 * 0 = obstacle, 1 = clear — no inversion needed.
 */
#define IR_OBS_READ_0() (((IR_OBS_GPIO_0->PDIR) >> IR_OBS_PIN_0) & 1u)
#define IR_OBS_READ_1() (((IR_OBS_GPIO_1->PDIR) >> IR_OBS_PIN_1) & 1u)
#define IR_OBS_READ_2() (((IR_OBS_GPIO_2->PDIR) >> IR_OBS_PIN_2) & 1u)
#define IR_OBS_READ_3() (((IR_OBS_GPIO_3->PDIR) >> IR_OBS_PIN_3) & 1u)
#define IR_OBS_READ_4() (((IR_OBS_GPIO_4->PDIR) >> IR_OBS_PIN_4) & 1u)
#define IR_OBS_READ_5() (((IR_OBS_GPIO_5->PDIR) >> IR_OBS_PIN_5) & 1u)

/* Backward-compatible alias for sensor 0 */
#define IR_OBS_READ()   IR_OBS_READ_0()

/*
 * TX-specific pin init — call after pin_config_init().
 * Configures all 6 IR obstacle sensor pins as GPIO inputs with pull-up.
 */
static inline void pin_config_tx_init(void)
{
    /* Sensor 0: PTB2 */
    IR_OBS_PORT_0->PCR[IR_OBS_PIN_0] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IR_OBS_GPIO_0->PDDR &= ~(1u << IR_OBS_PIN_0);

    /* Sensor 1: PTC0 */
    IR_OBS_PORT_1->PCR[IR_OBS_PIN_1] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IR_OBS_GPIO_1->PDDR &= ~(1u << IR_OBS_PIN_1);

    /* Sensor 2: PTC3 */
    IR_OBS_PORT_2->PCR[IR_OBS_PIN_2] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IR_OBS_GPIO_2->PDDR &= ~(1u << IR_OBS_PIN_2);

    /* Sensor 3: PTC4 */
    IR_OBS_PORT_3->PCR[IR_OBS_PIN_3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IR_OBS_GPIO_3->PDDR &= ~(1u << IR_OBS_PIN_3);

    /* Sensor 4: PTC5 */
    IR_OBS_PORT_4->PCR[IR_OBS_PIN_4] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IR_OBS_GPIO_4->PDDR &= ~(1u << IR_OBS_PIN_4);

    /* Sensor 5: PTC6 */
    IR_OBS_PORT_5->PCR[IR_OBS_PIN_5] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IR_OBS_GPIO_5->PDDR &= ~(1u << IR_OBS_PIN_5);
}

#endif /* PIN_CONFIG_TX_H */
