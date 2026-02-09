#ifndef SENSOR_SAMPLE_H
#define SENSOR_SAMPLE_H

#include <stdint.h>
#include "pin_config.h"

/*
 * Sensor Snapshot - Multi-sensor support (6 IR obstacle sensors)
 *
 * Contains real sensor readings transmitted from sensor board
 * to control board every 100ms.
 *
 * Payload layout (6 bytes):
 *   [0] ir[0]  — sensor 0 (PTB2):  0 = obstacle, 1 = clear
 *   [1] ir[1]  — sensor 1 (PTC0):  0 = obstacle, 1 = clear
 *   [2] ir[2]  — sensor 2 (PTC3):  0 = obstacle, 1 = clear
 *   [3] ir[3]  — sensor 3 (PTC4):  0 = obstacle, 1 = clear
 *   [4] ir[4]  — sensor 4 (PTC5):  0 = obstacle, 1 = clear
 *   [5] ir[5]  — sensor 5 (PTC6):  0 = obstacle, 1 = clear
 */

#define SNAPSHOT_SIZE  NUM_IR_SENSORS   /* 6 bytes */

typedef struct {
    uint8_t ir[NUM_IR_SENSORS];   /* 0 = obstacle, 1 = clear, per sensor */
} snapshot_t;

/*
 * Read all sensors into snapshot struct.
 * Call from main loop (not ISR).
 */
static inline void snapshot_sample(snapshot_t *s)
{
    s->ir[0] = (uint8_t)IR_OBS_READ_0();
    s->ir[1] = (uint8_t)IR_OBS_READ_1();
    s->ir[2] = (uint8_t)IR_OBS_READ_2();
    s->ir[3] = (uint8_t)IR_OBS_READ_3();
    s->ir[4] = (uint8_t)IR_OBS_READ_4();
    s->ir[5] = (uint8_t)IR_OBS_READ_5();
}

/*
 * Returns 1 if ANY sensor detects an obstacle, 0 if all clear.
 */
static inline uint8_t snapshot_any_obstacle(const snapshot_t *s)
{
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        if (s->ir[i] == 0)
            return 1;
    }
    return 0;
}

/*
 * Serialize snapshot to byte buffer for framing.
 * buf must be at least SNAPSHOT_SIZE bytes.
 */
static inline void snapshot_pack(const snapshot_t *s, uint8_t *buf)
{
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++)
        buf[i] = s->ir[i];
}

/*
 * Deserialize byte buffer back into snapshot struct.
 */
static inline void snapshot_unpack(snapshot_t *s, const uint8_t *buf)
{
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++)
        s->ir[i] = buf[i];
}

#endif /* SENSOR_SAMPLE_H */
