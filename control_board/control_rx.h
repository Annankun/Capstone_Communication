/*
 * control_rx.h - RX processing helpers for the control board
 *
 * Provides debug/statistics access for Phase 8 data collection.
 * The actual parsing happens inline in main.c using the parser.
 */
#ifndef CONTROL_RX_H
#define CONTROL_RX_H

#include <stdint.h>

/*
 * comms_stats_t - Communication statistics for diagnostics / reporting.
 *
 * Collect these during Phase 8 pressure testing.
 * Feed them into your Capstone report as "engineering evidence".
 */
typedef struct {
    uint32_t good_frames;
    uint32_t bad_crc;
    uint32_t sync_resets;
    uint32_t rx_overflow;
    uint32_t comms_lost_count;
    uint32_t max_gap_ms;          /* longest time between valid frames */
    uint32_t last_frame_time_ms;  /* millis() of last good frame       */
} comms_stats_t;

/*
 * comms_stats_update_gap - Call after every good frame to track max gap.
 */
static inline void comms_stats_update_gap(comms_stats_t *s, uint32_t now_ms)
{
    if (s->last_frame_time_ms != 0) {
        uint32_t gap = now_ms - s->last_frame_time_ms;
        if (gap > s->max_gap_ms)
            s->max_gap_ms = gap;
    }
    s->last_frame_time_ms = now_ms;
}

#endif /* CONTROL_RX_H */
