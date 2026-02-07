/*
 * motor_ctrl.h - Stub motor control interface
 *
 * Phase 6: Provides motor_stop() for fail-safe.
 *
 * TODO: Replace stubs with real PWM / H-bridge driver calls
 * for your specific motor driver hardware.
 */
#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <stdint.h>

/*
 * motor_init - Initialize motor driver GPIOs / PWM channels.
 * Call once at startup.
 */
static inline void motor_init(void)
{
    /* TODO: Configure TPM/PWM channels for your motor driver */
}

/*
 * motor_stop - Immediately set all motor outputs to 0.
 * This is the fail-safe action when communication is lost.
 */
static inline void motor_stop(void)
{
    /* TODO: Set PWM duty to 0 / disable motor driver enables */
}

/*
 * motor_set - Set motor speeds.
 * @left:  left motor duty (-100 to +100, negative = reverse)
 * @right: right motor duty (-100 to +100)
 */
static inline void motor_set(int8_t left, int8_t right)
{
    (void)left;
    (void)right;
    /* TODO: Map to PWM duty + direction pins */
}

#endif /* MOTOR_CTRL_H */
