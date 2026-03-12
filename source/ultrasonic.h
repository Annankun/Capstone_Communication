#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "MKL25Z4.h"
#include <stdint.h>

/* S3 BACK:  PTD0 (TRIG), PTD1 (ECHO) */
#define US3_TRIG_GPIO   GPIOD
#define US3_TRIG_PORT   PORTD
#define US3_TRIG_PIN    0u
#define US3_TRIG_MASK   (1u << US3_TRIG_PIN)
#define US3_ECHO_GPIO   GPIOD
#define US3_ECHO_PORT   PORTD
#define US3_ECHO_PIN    1u
#define US3_ECHO_MASK   (1u << US3_ECHO_PIN)

/* S1 LEFT:  PTD6 (TRIG), PTD7 (ECHO) — avoids UART2 on PTD2/PTD3 */
#define US1_TRIG_GPIO   GPIOD
#define US1_TRIG_PORT   PORTD
#define US1_TRIG_PIN    6u
#define US1_TRIG_MASK   (1u << US1_TRIG_PIN)
#define US1_ECHO_GPIO   GPIOD
#define US1_ECHO_PORT   PORTD
#define US1_ECHO_PIN    7u
#define US1_ECHO_MASK   (1u << US1_ECHO_PIN)

/* S2 RIGHT: PTD4 (TRIG), PTD5 (ECHO) */
#define US2_TRIG_GPIO   GPIOD
#define US2_TRIG_PORT   PORTD
#define US2_TRIG_PIN    4u
#define US2_TRIG_MASK   (1u << US2_TRIG_PIN)
#define US2_ECHO_GPIO   GPIOD
#define US2_ECHO_PORT   PORTD
#define US2_ECHO_PIN    5u
#define US2_ECHO_MASK   (1u << US2_ECHO_PIN)

void     Ultrasonic_InitAll(void);
uint32_t Ultrasonic_MeasureCm_Left(void);
uint32_t Ultrasonic_MeasureCm_Right(void);
uint32_t Ultrasonic_MeasureCm_Back(void);

#endif /* ULTRASONIC_H */
