#ifndef PIN_CONFIG_RX_H
#define PIN_CONFIG_RX_H

/*
 * RX (Control Board) Pin Configuration - FRDM-KL25Z
 *
 * Pins specific to the control / receiver board.
 * Include pin_config.h first for shared definitions.
 */

#include "pin_config.h"

/*
 * RX-specific pin init — call after pin_config_init().
 */
static inline void pin_config_rx_init(void)
{
    /* No RX-specific pins configured currently */
}

#endif /* PIN_CONFIG_RX_H */
