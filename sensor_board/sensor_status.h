// sensor_board/sensor_status.h
// （由同学们维护，你只是读）

#ifndef SENSOR_STATUS_H
#define SENSOR_STATUS_H

#include <stdint.h>

/*
 * 所有传感器的最终状态
 * 由同学们在后台持续更新
 * 你每100ms读一次，然后发送
 */
typedef struct {
    // 障碍物检测结果（都是 0/1）
    uint8_t ir[6];              // 0 = 有障碍, 1 = 清晰
    uint8_t ultrasonic[4];      // 0 = 危险, 1 = 安全
    uint8_t tof;                // 0 = 危险, 1 = 安全
    uint8_t gps_valid;          // 0 = 无定位, 1 = 有定位

    // GPS 坐标（如果需要）
    int32_t gps_lat;            // 纬度
    int32_t gps_lon;            // 经度

    // 填充
    uint8_t padding[2];
} sensor_status_t;

// 全局变量（同学们维护，你读）
extern sensor_status_t g_sensor_status;

/*
 * Payload size for framing (22 bytes):
 *   [0..5]   ir[0..5]
 *   [6..9]   ultrasonic[0..3]
 *   [10]     tof
 *   [11]     gps_valid
 *   [12..15] gps_lat  (big-endian: MSB first)
 *   [16..19] gps_lon  (big-endian: MSB first)
 *   [20..21] padding[0..1]
 */
#define SENSOR_STATUS_SIZE  22u

/*
 * Serialize sensor_status_t into a flat byte buffer for framing.
 * int32_t GPS fields are packed big-endian (MSB first) for reliable
 * cross-platform reconstruction on the control board.
 * buf must be at least SENSOR_STATUS_SIZE bytes.
 */
static inline void sensor_status_pack(const sensor_status_t *s, uint8_t *buf)
{
    uint8_t i;

    /* ir[6] */
    for (i = 0; i < 6u; i++)
        buf[i] = s->ir[i];

    /* ultrasonic[4] */
    for (i = 0; i < 4u; i++)
        buf[6u + i] = s->ultrasonic[i];

    /* tof, gps_valid */
    buf[10] = s->tof;
    buf[11] = s->gps_valid;

    /* gps_lat — big-endian: byte 12 = MSB, byte 15 = LSB */
    buf[12] = (uint8_t)((uint32_t)s->gps_lat >> 24);
    buf[13] = (uint8_t)((uint32_t)s->gps_lat >> 16);
    buf[14] = (uint8_t)((uint32_t)s->gps_lat >>  8);
    buf[15] = (uint8_t)((uint32_t)s->gps_lat      );

    /* gps_lon — big-endian: byte 16 = MSB, byte 19 = LSB */
    buf[16] = (uint8_t)((uint32_t)s->gps_lon >> 24);
    buf[17] = (uint8_t)((uint32_t)s->gps_lon >> 16);
    buf[18] = (uint8_t)((uint32_t)s->gps_lon >>  8);
    buf[19] = (uint8_t)((uint32_t)s->gps_lon      );

    /* padding */
    buf[20] = s->padding[0];
    buf[21] = s->padding[1];
}

/*
 * Deserialize a byte buffer back into sensor_status_t.
 * Mirrors sensor_status_pack() exactly.
 */
static inline void sensor_status_unpack(sensor_status_t *s, const uint8_t *buf)
{
    uint8_t i;

    for (i = 0; i < 6u; i++)
        s->ir[i] = buf[i];

    for (i = 0; i < 4u; i++)
        s->ultrasonic[i] = buf[6u + i];

    s->tof       = buf[10];
    s->gps_valid = buf[11];

    s->gps_lat = (int32_t)(((uint32_t)buf[12] << 24) |
                            ((uint32_t)buf[13] << 16) |
                            ((uint32_t)buf[14] <<  8) |
                             (uint32_t)buf[15]);

    s->gps_lon = (int32_t)(((uint32_t)buf[16] << 24) |
                            ((uint32_t)buf[17] << 16) |
                            ((uint32_t)buf[18] <<  8) |
                             (uint32_t)buf[19]);

    s->padding[0] = buf[20];
    s->padding[1] = buf[21];
}

/*
 * Returns 1 if ANY obstacle sensor (IR or ultrasonic or ToF) reports danger.
 */
static inline uint8_t sensor_status_any_obstacle(const sensor_status_t *s)
{
    uint8_t i;
    for (i = 0; i < 6u; i++) {
        if (s->ir[i] == 0) return 1;
    }
    for (i = 0; i < 4u; i++) {
        if (s->ultrasonic[i] == 0) return 1;
    }
    if (s->tof == 0) return 1;
    return 0;
}

#endif /* SENSOR_STATUS_H */
