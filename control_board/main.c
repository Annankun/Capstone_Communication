/*
 * Control Board - Step 5: 6x IR Obstacle Sensors + Emergency Stop
 *
 * UART2 RX interrupt pushes bytes into a ring buffer.
 * Main loop drains ring buffer into parser, unpacks validated frames
 * into a snapshot_t struct containing 6 real IR sensor readings.
 *
 * Emergency stop sources:
 *   1. ESTOP frame from sensor board (payload[0] = 1/0)
 *   2. Local PTB3 button on control board (toggle on press)
 *      - Press once:  enter estop (stop all activity, ignore data)
 *      - Press again: resume normal operation
 *   - Blue LED steady = emergency stop active
 *
 * Tracks:
 *   - good_frames:  valid frames with correct CRC
 *   - bad_crc:      frames with CRC mismatch
 *   - seq_errors:   out-of-order or lost sequence numbers
 *
 * LED feedback:
 *   - Green flash:  valid frame received
 *   - Red (steady): any obstacle detected by IR sensors
 *   - Blue:         safe mode (timeout) or emergency stop
 *
 * Debug output via UART0 (OpenSDA virtual COM) at 115200 baud.
 */

#include "MKL25Z4.h"
#include "pin_config_rx.h"
#include "uart.h"
#include "debug_uart.h"
#include "ringbuf.h"
#include "protocol.h"
#include "sensor_sample.h"
#include "motor_cmd.h"

/* ---- Global ring buffer and overflow counter (used by uart.h) ---- */

ringbuf_t           rx_ring;
volatile uint32_t   rx_overflow_count;
volatile uint32_t   hw_overrun_count;

/*
 * Global motor command — written by students' decision code, read by comms loop.
 *
 * Students set fields directly:
 *   g_motor_cmd.enable    = 1;
 *   g_motor_cmd.left_dir  = 1;   // 1=forward, 0=reverse
 *   g_motor_cmd.right_dir = 0;
 *
 * The 100ms TX loop below reads this and sends a FRAME_TYPE_MOTOR frame
 * to the Motor Board automatically — no extra calls needed.
 */
volatile motor_cmd_t g_motor_cmd;

/* ---- UART2 RX ISR ---- */

void UART2_IRQHandler(void)
{
    uint8_t status = COMM_UART->S1;

    if (status & UART_S1_RDRF_MASK) {
        uint8_t c = COMM_UART->D;    /* read clears RDRF */
        if (!ring_push(&rx_ring, c)) {
            rx_overflow_count++;      /* ring full - byte lost */
        }
    }

    /* Clear overrun flag if set (must read S1 then D, already done above) */
    if (status & UART_S1_OR_MASK) {
        (void)COMM_UART->D;          /* clear OR flag */
        hw_overrun_count++;
    }
}

/* ---- Simple delay using SysTick ---- */

static volatile uint32_t ms_ticks;

void SysTick_Handler(void)
{
    ms_ticks++;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = ms_ticks;
    while ((ms_ticks - start) < ms)
        ;
}

/* ---- Decimal print helper ---- */

static void debug_putdec(uint32_t n)
{
    char tmp[10];
    int  i = 0;

    if (n == 0) {
        debug_putchar('0');
        return;
    }
    while (n > 0) {
        tmp[i++] = '0' + (char)(n % 10);
        n /= 10;
    }
    while (i > 0) {
        debug_putchar(tmp[--i]);
    }
}

/* ---- Main ---- */

int main(void)
{
    uint32_t good_frames   = 0;
    uint32_t bad_crc       = 0;
    uint32_t seq_errors    = 0;
    uint32_t rx_byte_count = 0;
    uint32_t last_report   = 0;
    uint32_t last_valid_rx = 0;   /* ms_ticks when last good frame received */
    uint8_t  expected_seq  = 0;
    uint8_t  first_frame   = 1;   /* flag: haven't received any frame yet */
    uint8_t  in_safe_mode  = 0;
    uint8_t  in_estop      = 0;   /* emergency stop active */

    /* Local button (PTB3) debounce state */
    uint8_t  btn_last      = 1;   /* last stable reading (1 = released) */
    uint32_t btn_debounce  = 0;   /* timestamp of last edge */
    #define  BTN_DEBOUNCE_MS 50u

    /* Motor TX state — sends g_motor_cmd to Motor Board every 100ms */
    uint8_t  motor_seq      = 0;
    uint32_t last_motor_tx  = 0;
    #define  MOTOR_TX_PERIOD_MS 100u
    uint8_t  motor_payload[MOTOR_CMD_SIZE];
    uint8_t  motor_frame[FRAME_HEADER_SIZE + MOTOR_CMD_SIZE + FRAME_CRC_SIZE];

    snapshot_t latest_snapshot;
    for (uint8_t i = 0; i < IR_OBS_COUNT; i++)
        latest_snapshot.ir_obs[i] = 1;   /* default: clear (no obstacle) */
    latest_snapshot.reserved[0] = 0;
    latest_snapshot.reserved[1] = 0;

    parser_t parser;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    motor_cmd_init(&g_motor_cmd);   /* safe stopped state before loop starts */

    pin_config_init();
    pin_config_rx_init();
    uart2_init();       /* enables RX ISR + ring buffer */
    debug_uart_init();

    RGB_ALL_OFF();
    parser_init(&parser);

    /* Startup blink: 3x blue to confirm board is alive */
    for (int blink = 0; blink < 3; blink++) {
        RGB_BLUE_ON();
        delay_ms(150);
        RGB_BLUE_OFF();
        delay_ms(150);
    }

    PRINTF("[CONTROL] Step 5: Receiving 6x IR sensor snapshots.\r\n");
    PRINTF("[CONTROL] UART2 RX on PTD2, TX on PTD3, 9600 baud\r\n");

    while (1) {
        uint8_t c;

        /* ---- Local ESTOP button (PTB3) - toggle on press ---- */
        {
            uint8_t btn_now = ESTOP_BTN_READ();
            if (btn_now != btn_last &&
                (ms_ticks - btn_debounce) >= BTN_DEBOUNCE_MS) {
                btn_debounce = ms_ticks;
                btn_last = btn_now;

                if (btn_now == 0) {          /* falling edge = press */
                    in_estop = !in_estop;    /* toggle */
                    if (in_estop) {
                        RGB_ALL_OFF();
                        RGB_BLUE_ON();
                        PRINTF("[CONTROL] *** ESTOP (button) ACTIVATED ***\r\n");
                    } else {
                        RGB_BLUE_OFF();
                        PRINTF("[CONTROL] *** ESTOP (button) RELEASED ***\r\n");
                    }
                }
            }
        }

        /* While in estop, flush incoming bytes but don't process frames */
        if (in_estop) {
            while (uart2_getchar(&c)) {
                /* discard */
            }
            parser_init(&parser);  /* reset parser so we start clean on resume */
            RGB_GREEN_OFF();
        } else {
            /* Drain ring buffer immediately into parser - no delays */
            while (uart2_getchar(&c)) {
                rx_byte_count++;

                int result = parser_feed(&parser, c);

                if (result == PARSE_OK) {
                    good_frames++;
                    last_valid_rx = ms_ticks;

                    /* Sequence check */
                    if (first_frame) {
                        expected_seq = parser.seq + 1;
                        first_frame = 0;
                    } else {
                        if (parser.seq != expected_seq)
                            seq_errors++;
                        expected_seq = parser.seq + 1;
                    }

                    /* Handle emergency stop frame from sensor board */
                    if (parser.type == FRAME_TYPE_ESTOP && parser.len >= 1) {
                        if (parser.payload[0]) {
                            in_estop = 1;
                            RGB_ALL_OFF();
                            RGB_BLUE_ON();
                            PRINTF("[CONTROL] *** ESTOP (remote) ACTIVATED ***\r\n");
                        } else {
                            in_estop = 0;
                            RGB_BLUE_OFF();
                            PRINTF("[CONTROL] *** ESTOP (remote) RELEASED ***\r\n");
                        }
                    }

                    /* Unpack snapshot if sensor frame with correct size */
                    if (parser.type == FRAME_TYPE_SENSOR &&
                        parser.len == SNAPSHOT_SIZE) {
                        snapshot_unpack(&latest_snapshot, parser.payload);
                    }

                    /* Exit safe mode if we were in it */
                    if (in_safe_mode) {
                        in_safe_mode = 0;
                        if (!in_estop)
                            RGB_BLUE_OFF();
                    }

                    /* Show obstacle status on red LED (not during estop) */
                    if (!in_estop) {
                        if (snapshot_any_obstacle(&latest_snapshot)) {
                            RGB_RED_ON();    /* obstacle detected */
                        } else {
                            RGB_RED_OFF();   /* all clear */
                        }
                    }

                    /* Brief green flash for valid frame */
                    RGB_GREEN_ON();
                } else if (result == PARSE_BAD_CRC) {
                    bad_crc++;
                }
            }

            /* Turn off green after drain (non-blocking visual feedback) */
            RGB_GREEN_OFF();
        }

        /* Heartbeat: 500ms without valid frame = safe mode (skip if estop) */
        if (!first_frame && !in_safe_mode && !in_estop &&
            (ms_ticks - last_valid_rx) >= 500u) {
            in_safe_mode = 1;
            RGB_BLUE_ON();
            RGB_RED_OFF();
            PRINTF("[CONTROL] SAFE MODE: no valid frame for 500ms!\r\n");
        }

        /* ---- Motor TX: send g_motor_cmd to Motor Board every 100ms ---- */
        if ((ms_ticks - last_motor_tx) >= MOTOR_TX_PERIOD_MS) {
            last_motor_tx = ms_ticks;

            /*
             * Read the global motor command (written by students' code),
             * pack into frame, and transmit via UART2 TX to Motor Board.
             * In estop or safe mode, force enable=0 so motors stop.
             */
            motor_cmd_pack(&g_motor_cmd, motor_payload);
            if (in_estop || in_safe_mode)
                motor_payload[0] = 0;   /* override enable to 0 */

            uint8_t frame_len = frame_pack(motor_frame, FRAME_TYPE_MOTOR,
                                           motor_seq, motor_payload,
                                           MOTOR_CMD_SIZE);
            for (uint8_t i = 0; i < frame_len; i++)
                uart2_putchar(motor_frame[i]);

            motor_seq++;
        }

        /* Periodic status report every 5 seconds */
        if ((ms_ticks - last_report) >= 5000u) {
            last_report = ms_ticks;

            PRINTF("[STATS] good=");
            debug_putdec(good_frames);
            PRINTF(" bad_crc=");
            debug_putdec(bad_crc);
            PRINTF(" seq_err=");
            debug_putdec(seq_errors);
            PRINTF(" ir=");
            for (uint8_t i = 0; i < IR_OBS_COUNT; i++)
                debug_putchar(latest_snapshot.ir_obs[i] ? '1' : '0');
            PRINTF(" overflow=");
            debug_putdec(rx_overflow_count);
            PRINTF(" hw_overrun=");
            debug_putdec(hw_overrun_count);
            PRINTF(" estop=");
            debug_putdec(in_estop);
            PRINTF("\r\n");
        }
    }
}
