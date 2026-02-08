#ifndef SENSOR_SAMPLE_H
#define SENSOR_SAMPLE_H

#include <stdint.h>
#include "pin_config.h"

/*
 * Sensor Snapshot - Step 4
 *
 * Contains real sensor readings transmitted from sensor board
 * to control board every 100ms.
 *
 * Payload layout (2 bytes):
 *   [0] ir_obstacle   — 0 = obstacle detected, 1 = clear
 *   [1] reserved      — 0x00 (for future sensors)
 */

#define SNAPSHOT_SIZE  2u

typedef struct {
    uint8_t ir_obstacle;    /* 0 = obstacle, 1 = clear (PTB2) */
    uint8_t reserved;       /* pad to fixed size, future use  */
} snapshot_t;

/*
 * Read all sensors into snapshot struct.
 * Call from main loop (not ISR).
 */
static inline void snapshot_sample(snapshot_t *s)
{
    s->ir_obstacle = (uint8_t)IR_OBS_READ();
    s->reserved    = 0x00;
}

/*
 * Serialize snapshot to byte buffer for framing.
 * buf must be at least SNAPSHOT_SIZE bytes.
 */
static inline void snapshot_pack(const snapshot_t *s, uint8_t *buf)
{
    buf[0] = s->ir_obstacle;
    buf[1] = s->reserved;
}

/*
 * Deserialize byte buffer back into snapshot struct.
 */
static inline void snapshot_unpack(snapshot_t *s, const uint8_t *buf)
{
    s->ir_obstacle = buf[0];
    s->reserved    = buf[1];
}

#endif /* SENSOR_SAMPLE_H */
