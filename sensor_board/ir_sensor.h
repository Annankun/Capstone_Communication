/*
 * ir_sensor.h — IR Obstacle Sensor Driver
 *
 * Hardware mapping (FRDM-KL25Z):
 *   Index   Pin     Port/GPIO   Active-low (0 = obstacle, 1 = clear)
 *     0     PTE3    PORTE bit3
 *     1     PTE2    PORTE bit2
 *     2     PTB11   PORTB bit11
 *     3     PTB10   PORTB bit10
 *     4     PTB9    PORTB bit9
 *     5     PTB8    PORTB bit8
 *
 * Template for teammates: follow the same two-function pattern
 * (xxx_init / xxx_read) for ultrasonic, ToF, and GPS drivers.
 */

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "MKL25Z4.h"
#include "pin_config.h"
#include "sensor_status.h"

/* Configure all 6 IR GPIO pins as inputs with pull-up. */
static inline void ir_sensor_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK;

    PORTE->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTE->PCR[2] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOE->PDDR  &= ~((1u << 3) | (1u << 2));

    PORTB->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[9]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[8]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOB->PDDR   &= ~((1u << 11) | (1u << 10) | (1u << 9) | (1u << 8));
}

/*
 * Read all 6 IR sensors from a single GPIO snapshot.
 * obs[0..5]: 0 = obstacle, 1 = clear
 */
static inline void ir_sensor_read(uint8_t obs[IR_COUNT])
{
    uint32_t pdir_e = GPIOE->PDIR;   /* snapshot PORTE */
    uint32_t pdir_b = GPIOB->PDIR;   /* snapshot PORTB */

    obs[0] = (uint8_t)((pdir_e >> 3)  & 1u);   /* PTE3  */
    obs[1] = (uint8_t)((pdir_e >> 2)  & 1u);   /* PTE2  */
    obs[2] = (uint8_t)((pdir_b >> 11) & 1u);   /* PTB11 */
    obs[3] = (uint8_t)((pdir_b >> 10) & 1u);   /* PTB10 */
    obs[4] = (uint8_t)((pdir_b >> 9)  & 1u);   /* PTB9  */
    obs[5] = (uint8_t)((pdir_b >> 8)  & 1u);   /* PTB8  */
}

#endif /* IR_SENSOR_H */
