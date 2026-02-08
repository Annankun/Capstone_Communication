#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

/*
 * Pin Configuration - FRDM-KL25Z
 *
 * Onboard RGB LED and UART2 serial pin assignments.
 */

/* ---- Onboard RGB LED (active-low) ---- */
#define RGB_RED_PORT    PORTB
#define RGB_RED_GPIO    GPIOB
#define RGB_RED_PIN     18      /* PTB18 - TPM2_CH0 */

#define RGB_GREEN_PORT  PORTB
#define RGB_GREEN_GPIO  GPIOB
#define RGB_GREEN_PIN   19      /* PTB19 - TPM2_CH1 */

#define RGB_BLUE_PORT   PORTD
#define RGB_BLUE_GPIO   GPIOD
#define RGB_BLUE_PIN    1       /* PTD1  - TPM0_CH1 */

/* ---- UART2 Serial (board-to-board comm) ---- */
#define COMM_UART           UART2
#define COMM_UART_IRQn      UART2_IRQn
#define COMM_UART_IRQHandler UART2_IRQHandler

#define COMM_TX_PORT    PORTD
#define COMM_TX_PIN     3       /* PTD3 - UART2_TX (ALT3) */

#define COMM_RX_PORT    PORTD
#define COMM_RX_PIN     2       /* PTD2 - UART2_RX (ALT3) */

#define COMM_UART_ALT   3       /* Port mux ALT3 for UART2 */

#define COMM_BAUD_RATE  9600

/* ---- IR Obstacle Sensor (MH-sensor, LM393, digital out) ---- */
#define IR_OBS_PORT     PORTB
#define IR_OBS_GPIO     GPIOB
#define IR_OBS_PIN      2       /* PTB2 - digital input from VOUT */

/*
 * Pin init helper — call once from main() before using UART or RGB.
 */
static inline void pin_config_init(void)
{
    /* Enable clock gating for Port B and Port D */
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

    /* --- RGB LED pins: GPIO output --- */
    RGB_RED_PORT->PCR[RGB_RED_PIN]   = PORT_PCR_MUX(1);
    RGB_GREEN_PORT->PCR[RGB_GREEN_PIN] = PORT_PCR_MUX(1);
    RGB_BLUE_PORT->PCR[RGB_BLUE_PIN] = PORT_PCR_MUX(1);

    RGB_RED_GPIO->PDDR   |= (1u << RGB_RED_PIN);
    RGB_GREEN_GPIO->PDDR |= (1u << RGB_GREEN_PIN);
    RGB_BLUE_GPIO->PDDR  |= (1u << RGB_BLUE_PIN);

    /* Start with all LEDs off (active-low: set high = off) */
    RGB_RED_GPIO->PSOR   = (1u << RGB_RED_PIN);
    RGB_GREEN_GPIO->PSOR = (1u << RGB_GREEN_PIN);
    RGB_BLUE_GPIO->PSOR  = (1u << RGB_BLUE_PIN);

    /* --- IR obstacle sensor: GPIO input with pull-up --- */
    IR_OBS_PORT->PCR[IR_OBS_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IR_OBS_GPIO->PDDR &= ~(1u << IR_OBS_PIN);  /* input */

    /* --- UART2 pins: ALT3 mux --- */
    COMM_TX_PORT->PCR[COMM_TX_PIN] = PORT_PCR_MUX(COMM_UART_ALT);
    COMM_RX_PORT->PCR[COMM_RX_PIN] = PORT_PCR_MUX(COMM_UART_ALT);

    /* Enable UART2 clock */
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
}

/* ---- RGB convenience macros (active-low) ---- */
#define RGB_RED_ON()     (RGB_RED_GPIO->PCOR   = (1u << RGB_RED_PIN))
#define RGB_RED_OFF()    (RGB_RED_GPIO->PSOR   = (1u << RGB_RED_PIN))
#define RGB_GREEN_ON()   (RGB_GREEN_GPIO->PCOR = (1u << RGB_GREEN_PIN))
#define RGB_GREEN_OFF()  (RGB_GREEN_GPIO->PSOR = (1u << RGB_GREEN_PIN))
#define RGB_BLUE_ON()    (RGB_BLUE_GPIO->PCOR  = (1u << RGB_BLUE_PIN))
#define RGB_BLUE_OFF()   (RGB_BLUE_GPIO->PSOR  = (1u << RGB_BLUE_PIN))

/* ---- IR sensor read (returns 0 = obstacle, 1 = clear) ---- */
#define IR_OBS_READ()  (((IR_OBS_GPIO->PDIR) >> IR_OBS_PIN) & 1u)

#define RGB_ALL_OFF() do { \
    RGB_RED_OFF(); RGB_GREEN_OFF(); RGB_BLUE_OFF(); \
} while (0)

#endif /* PIN_CONFIG_H */
