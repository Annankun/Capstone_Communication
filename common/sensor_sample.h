#ifndef SENSOR_SAMPLE_H
#define SENSOR_SAMPLE_H

#include <stdint.h>
#include "pin_config.h"

/*
 * Sensor Snapshot — 6x IR Obstacle Sensors
 *
 * Contains real sensor readings transmitted from sensor board
 * to control board every 100ms.
 *
 * Payload layout (8 bytes):
 *   [0..5] ir_obs[0..5]  — 0 = obstacle detected, 1 = clear
 *   [6..7] reserved      — 0x00 (for future sensors)
 *
 * Sensor index mapping:
 *   [0] PTE3   [1] PTE2   [2] PTB11
 *   [3] PTB10  [4] PTB9   [5] PTB8
 */

#define SNAPSHOT_SIZE  8u

typedef struct {
    uint8_t ir_obs[IR_OBS_COUNT];  /* 0 = obstacle, 1 = clear */
    uint8_t reserved[2];           /* pad to fixed size, future use */
} snapshot_t;

/*
 * Read all sensors into snapshot struct.
 * Call from main loop (not ISR).
 */
static inline void snapshot_sample(snapshot_t *s)
{
    ir_obs_read_all(s->ir_obs);
    s->reserved[0] = 0x00;
    s->reserved[1] = 0x00;
}

/*
 * Serialize snapshot to byte buffer for framing.
 * buf must be at least SNAPSHOT_SIZE bytes.
 */
static inline void snapshot_pack(const snapshot_t *s, uint8_t *buf)
{
    for (uint8_t i = 0; i < IR_OBS_COUNT; i++)
        buf[i] = s->ir_obs[i];
    buf[6] = s->reserved[0];
    buf[7] = s->reserved[1];
}

/*
 * Deserialize byte buffer back into snapshot struct.
 */
static inline void snapshot_unpack(snapshot_t *s, const uint8_t *buf)
{
    for (uint8_t i = 0; i < IR_OBS_COUNT; i++)
        s->ir_obs[i] = buf[i];
    s->reserved[0] = buf[6];
    s->reserved[1] = buf[7];
}

/*
 * Returns 1 if ANY obstacle sensor reads 0 (obstacle detected).
 */
static inline uint8_t snapshot_any_obstacle(const snapshot_t *s)
{
    for (uint8_t i = 0; i < IR_OBS_COUNT; i++) {
        if (s->ir_obs[i] == 0)
            return 1;
    }
    return 0;
}

#endif /* SENSOR_SAMPLE_H */
