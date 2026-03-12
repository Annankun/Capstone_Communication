#include "MKL25Z4.h"

extern void     Init_PIT0_10us(void); /* from timers.c */
extern void     Start_PIT0(void);     /* from timers.c */
extern unsigned Timer_Micros(void);   /* from timers.c */

/* S3 BACK:  PTD0 (TRIG), PTD1 (ECHO) */
#define US3_TRIG_GPIO   GPIOD
#define US3_TRIG_PORT   PORTD
#define US3_TRIG_PIN    0u
#define US3_TRIG_MASK   (1u << US3_TRIG_PIN)
#define US3_ECHO_GPIO   GPIOD
#define US3_ECHO_PORT   PORTD
#define US3_ECHO_PIN    1u
#define US3_ECHO_MASK   (1u << US3_ECHO_PIN)

/* S1 LEFT:  PTD6 (TRIG), PTD7 (ECHO) — moved from PTD2/PTD3 (UART2 conflict) */
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

static void delay_cycles(volatile uint32_t n)
{
    while (n--) {
        __NOP();
    }
}

static volatile uint32_t g_active_echoMask = 0;
static volatile uint32_t g_echo_rise_us = 0;
static volatile uint32_t g_echo_fall_us = 0;
static volatile uint8_t  g_got_rise = 0;
static volatile uint8_t  g_done = 0;

void PORTD_IRQHandler(void)
{
    uint32_t flags = PORTD->ISFR;

    if (flags & g_active_echoMask) {
        uint32_t now = Timer_Micros();

        if (GPIOD->PDIR & g_active_echoMask) {
            g_echo_rise_us = now;
            g_got_rise = 1;
        } else {
            if (g_got_rise) {
                g_echo_fall_us = now;
                g_done = 1;
            }
        }
    }

    PORTD->ISFR = flags;
}

static uint32_t MeasureUs_Single(GPIO_Type *trigGPIO,
                                 uint32_t trigMask,
                                 GPIO_Type *echoGPIO,
                                 uint32_t echoMask)
{
    (void)echoGPIO;
    uint32_t start_us;

    trigGPIO->PCOR = trigMask;
    delay_cycles(3000);

    __disable_irq();
    g_active_echoMask = echoMask;
    g_echo_rise_us = 0;
    g_echo_fall_us = 0;
    g_got_rise = 0;
    g_done = 0;
    start_us = Timer_Micros();
    __enable_irq();

    trigGPIO->PSOR = trigMask;
    delay_cycles(3000);
    trigGPIO->PCOR = trigMask;

    while (!g_got_rise) {
        if ((Timer_Micros() - start_us) > 5000u) {
            g_active_echoMask = 0;
            return 0u;
        }
        __WFI();
    }

    while (!g_done) {
        if ((Timer_Micros() - start_us) > 5000u) {
            g_active_echoMask = 0;
            return 0u;
        }
        __WFI();
    }

    g_active_echoMask = 0;
    return (uint32_t)(g_echo_fall_us - g_echo_rise_us);
}

void Ultrasonic_InitAll(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    Init_PIT0_10us();
    Start_PIT0();

    /* S3 BACK: PTD0/PTD1 */
    US3_TRIG_PORT->PCR[US3_TRIG_PIN] = PORT_PCR_MUX(1);
    US3_TRIG_GPIO->PDDR |= US3_TRIG_MASK;
    US3_ECHO_PORT->PCR[US3_ECHO_PIN] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0xB);
    US3_ECHO_GPIO->PDDR &= ~US3_ECHO_MASK;

    /* S1 LEFT: PTD6/PTD7 */
    US1_TRIG_PORT->PCR[US1_TRIG_PIN] = PORT_PCR_MUX(1);
    US1_TRIG_GPIO->PDDR |= US1_TRIG_MASK;
    US1_ECHO_PORT->PCR[US1_ECHO_PIN] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0xB);
    US1_ECHO_GPIO->PDDR &= ~US1_ECHO_MASK;

    /* S2 RIGHT: PTD4/PTD5 */
    US2_TRIG_PORT->PCR[US2_TRIG_PIN] = PORT_PCR_MUX(1);
    US2_TRIG_GPIO->PDDR |= US2_TRIG_MASK;
    US2_ECHO_PORT->PCR[US2_ECHO_PIN] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0xB);
    US2_ECHO_GPIO->PDDR &= ~US2_ECHO_MASK;

    US3_TRIG_GPIO->PCOR = US3_TRIG_MASK;
    US1_TRIG_GPIO->PCOR = US1_TRIG_MASK;
    US2_TRIG_GPIO->PCOR = US2_TRIG_MASK;

    PORTD->ISFR = 0xFFFFFFFFu;
    NVIC_ClearPendingIRQ(PORTD_IRQn);
    NVIC_EnableIRQ(PORTD_IRQn);
}

uint32_t Ultrasonic_MeasureCm_Left(void)
{
    uint32_t us = MeasureUs_Single(
        US1_TRIG_GPIO, US1_TRIG_MASK,
        US1_ECHO_GPIO, US1_ECHO_MASK);

    return us ? (us / 58u) : 0u;
}

uint32_t Ultrasonic_MeasureCm_Right(void)
{
    uint32_t us = MeasureUs_Single(
        US2_TRIG_GPIO, US2_TRIG_MASK,
        US2_ECHO_GPIO, US2_ECHO_MASK);

    return us ? (us / 58u) : 0u;
}

uint32_t Ultrasonic_MeasureCm_Back(void)
{
    uint32_t us = MeasureUs_Single(
        US3_TRIG_GPIO, US3_TRIG_MASK,
        US3_ECHO_GPIO, US3_ECHO_MASK);

    return us ? (us / 58u) : 0u;
}
