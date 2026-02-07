/*
 * sensor_sample.h - Snapshot v1 structure and sampling function
 *
 * Phase 4+: Defines the data that gets packed into each frame.
 * Start with the most critical fields; add more later.
 *
 * IMPORTANT: This struct is memcpy'd atomically before packing,
 * so all fields represent the same point in time.
 */
#ifndef SENSOR_SAMPLE_H
#define SENSOR_SAMPLE_H

#include <stdint.h>

/*
 * snapshot_v1_t - Sensor data snapshot
 *
 * Keep this struct small.  Every byte adds to frame size and TX time.
 * Current size: 16 bytes.
 *
 * Phase 7: Add fields one at a time and re-test each time!
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;       /* millis() at sample time         */
    uint8_t  seq;                /* frame sequence number           */
    uint8_t  line_value;         /* line sensor reading (0-255)     */
    uint16_t obstacle_distance;  /* ultrasonic distance in mm       */
    int16_t  enc_left;           /* left encoder ticks since last   */
    int16_t  enc_right;          /* right encoder ticks since last  */
    uint8_t  _reserved[4];      /* pad to 16 bytes; use for future */
} snapshot_v1_t;

/*
 * sensor_sample - Read all sensors and fill the snapshot struct.
 *
 * TODO (Phase 7): Replace stub values with real sensor reads.
 * For Phase 4 testing, this returns incrementing dummy values
 * so you can verify all fields update consistently.
 */
static inline void sensor_sample(snapshot_v1_t *snap)
{
    /* ---- STUB: replace with real sensor reads ---- */
    static uint8_t  dummy_line = 0;
    static uint16_t dummy_dist = 100;
    static int16_t  dummy_enc  = 0;

    snap->line_value        = dummy_line++;
    snap->obstacle_distance = dummy_dist;
    snap->enc_left          = dummy_enc;
    snap->enc_right         = dummy_enc;

    dummy_dist += 1;
    if (dummy_dist > 4000) dummy_dist = 100;
    dummy_enc++;
    /* timestamp_ms and seq are set by the caller */
}

#endif /* SENSOR_SAMPLE_H */
