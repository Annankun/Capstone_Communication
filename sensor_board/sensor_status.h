// sensor_board/sensor_status.h
// （由同学们维护，你只是读）

#ifndef SENSOR_STATUS_H
#define SENSOR_STATUS_H

#include <stdint.h>
#include <string.h>

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
 * Payload size for framing.
 * Layout (22 bytes):
 *   [0..5]   ir[0..5]
 *   [6..9]   ultrasonic[0..3]
 *   [10]     tof
 *   [11]     gps_valid
 *   [12..15] gps_lat  (little-endian)
 *   [16..19] gps_lon  (little-endian)
 *   [20..21] padding[0..1]
 */
#define SENSOR_STATUS_SIZE  22u

/*
 * Serialize sensor_status_t into a flat byte buffer for framing.
 * buf must be at least SENSOR_STATUS_SIZE bytes.
 */
static inline void sensor_status_pack(const sensor_status_t *s, uint8_t *buf)
{
    memcpy(buf, s, SENSOR_STATUS_SIZE);
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
