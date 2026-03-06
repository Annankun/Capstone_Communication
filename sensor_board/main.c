/*
 * Sensor Board — TX
 *
 * Main loop (10 Hz):
 *   update_sensor_status() → snapshot_sample() → snapshot_pack()
 *   → frame_pack() → UART2 send
 *
 * Debug (UART0, 115200): prints sensor values every 10 frames.
 */

#include "MKL25Z4.h"
#include "ir_sensor.h"
#include "uart.h"
#include "debug_uart.h"
#include "ringbuf.h"
#include "protocol.h"
#include "sensor_status.h"
#include "sensor_sample.h"

sensor_status_t g_sensor_status;

/* Globals required by uart.h (TX board discards incoming bytes) */
ringbuf_t         rx_ring;
volatile uint32_t rx_overflow_count;

void UART2_IRQHandler(void)
{
    uint8_t status = COMM_UART->S1;
    if (status & (UART_S1_RDRF_MASK | UART_S1_OR_MASK))
        (void)COMM_UART->D;   /* discard */
}

/* ---- SysTick: 1 ms ---- */

static volatile uint32_t ms_ticks;

void SysTick_Handler(void) { ms_ticks++; }

static void delay_ms(uint32_t ms)
{
    uint32_t start = ms_ticks;
    while ((ms_ticks - start) < ms)
        ;
}

/* ====================================================================
 * Sensor driver stubs — replace bodies with real driver calls
 * ==================================================================== */

static void ultrasonic_init(void) { /* TODO */ }

static void ultrasonic_read(uint8_t obs[US_COUNT])
{
    /* TODO: trigger measurement, threshold to 0/1 */
    uint8_t i;
    for (i = 0; i < US_COUNT; i++)
        obs[i] = 1;
}

static void tof_init(void) { /* TODO */ }

static void tof_read(uint8_t *obstacle)
{
    /* TODO: read distance; set *obstacle = (dist_mm < threshold) ? 0 : 1 */
    *obstacle = 1;
}

static void gps_init(void) { /* TODO */ }

static void gps_read(uint8_t *valid, int32_t *lat_deg7, int32_t *lon_deg7)
{
    /* TODO: parse latest NMEA sentence */
    *valid    = 0;
    *lat_deg7 = 0;
    *lon_deg7 = 0;
}

/* ====================================================================
 * sensors_init_all / update_sensor_status
 * ==================================================================== */

static void sensors_init_all(void)
{
    ir_sensor_init();
    ultrasonic_init();
    tof_init();
    gps_init();
}

static void update_sensor_status(void)
{
    ir_sensor_read(g_sensor_status.ir_obs);
    ultrasonic_read(g_sensor_status.us_obs);
    tof_read(&g_sensor_status.tof_obstacle);
    gps_read(&g_sensor_status.gps_valid,
             &g_sensor_status.lat_deg7,
             &g_sensor_status.lon_deg7);
}

/* ====================================================================
 * debug_print_tx
 * Format: [TX] IR=110010 US=0011 TOF=1 GPS=1 LAT=+374230000 LON=-1220840000
 * ==================================================================== */

static void debug_putdec32(int32_t n)
{
    char tmp[11];
    int  i = 0;
    uint32_t uval;

    if (n < 0) {
        debug_putchar('-');
        /* avoid UB on INT32_MIN */
        uval = (uint32_t)(-(n + 1)) + 1u;
    } else {
        debug_putchar('+');
        uval = (uint32_t)n;
    }

    if (uval == 0) { debug_putchar('0'); return; }
    while (uval > 0) { tmp[i++] = '0' + (char)(uval % 10); uval /= 10; }
    while (i > 0) debug_putchar(tmp[--i]);
}

static void debug_print_tx(const snapshot_t *s)
{
    uint8_t i;

    PRINTF("[TX] IR=");
    for (i = 0; i < IR_COUNT; i++) debug_putchar(s->ir_obs[i] ? '1' : '0');
    PRINTF(" US=");
    for (i = 0; i < US_COUNT; i++) debug_putchar(s->us_obs[i] ? '1' : '0');
    PRINTF(" TOF="); debug_putchar(s->tof_obstacle ? '1' : '0');
    PRINTF(" GPS="); debug_putchar(s->gps_valid ? '1' : '0');
    PRINTF(" LAT="); debug_putdec32(s->lat_deg7);
    PRINTF(" LON="); debug_putdec32(s->lon_deg7);
    PRINTF("\r\n");
}

/* ====================================================================
 * main
 * ==================================================================== */

int main(void)
{
    uint8_t    seq        = 0;
    uint8_t    debug_ctr  = 0;
    snapshot_t snap;
    uint8_t    payload[SNAPSHOT_PAYLOAD_BYTES];
    uint8_t    frame_buf[FRAME_HEADER_SIZE + SNAPSHOT_PAYLOAD_BYTES + FRAME_CRC_SIZE];

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    pin_config_init();
    sensors_init_all();
    uart2_init();
    debug_uart_init();

    RGB_ALL_OFF();

    /* Startup blink: 3x green */
    for (int b = 0; b < 3; b++) {
        RGB_GREEN_ON();  delay_ms(150);
        RGB_GREEN_OFF(); delay_ms(150);
    }

    PRINTF("[SENSOR] Sensor board ready. Polling all sensors at 10 Hz.\r\n");

    while (1) {
        /* 1. Read all sensors → g_sensor_status */
        update_sensor_status();

        /* 2. Copy g_sensor_status into snapshot */
        snapshot_sample(&snap);

        /* 3. Red LED = any obstacle */
        if (snapshot_any_obstacle(&snap))
            RGB_RED_ON();
        else
            RGB_RED_OFF();

        /* 4. Serialize → frame → transmit */
        snapshot_pack(&snap, payload);
        uint8_t frame_len = frame_pack(frame_buf, FRAME_TYPE_SENSOR, seq,
                                       payload, SNAPSHOT_PAYLOAD_BYTES);
        uint8_t i;
        for (i = 0; i < frame_len; i++)
            uart2_putchar(frame_buf[i]);

        seq++;

        /* 5. Green flash: frame sent */
        RGB_GREEN_ON();  delay_ms(10);
        RGB_GREEN_OFF();

        /* 6. Debug print every 10 frames */
        if (++debug_ctr >= 10) {
            debug_ctr = 0;
            debug_print_tx(&snap);
        }

        /* 7. Wait for next cycle */
        delay_ms(90);
    }
}
