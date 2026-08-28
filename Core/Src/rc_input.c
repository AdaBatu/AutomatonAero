/**
 * @file rc_input.c
 * @brief RC receiver PWM input capture implementation
 * 
 * Uses GPIO EXTI interrupts to measure PWM pulse widths from RC receiver.
 * Pin assignments:
 *   PB1  - Throttle
 *   PB2  - Roll
 *   PC12 - Pitch
 *   PB3  - Yaw / front wheel steering PWM
 */
#include "rc_input.h"
#include <string.h>
#include <math.h>

/* Pin to channel mapping */
#define PIN_THROTTLE    GPIO_PIN_1
#define PIN_ROLL        GPIO_PIN_2
#define PIN_PITCH       GPIO_PIN_12
#define PIN_YAW         GPIO_PIN_3

static GPIO_TypeDef *RC_ChannelPort(uint8_t channel)
{
    return (channel == RC_CH_PITCH) ? GPIOC : GPIOB;
}

/* Get channel index from GPIO pin */
static int8_t RC_PinToChannel(uint16_t pin)
{
    switch (pin) {
        case PIN_THROTTLE: return RC_CH_THROTTLE;
        case PIN_ROLL:     return RC_CH_ROLL;
        case PIN_PITCH:    return RC_CH_PITCH;
        case PIN_YAW:      return RC_CH_YAW;
        default:           return -1;
    }
}

/* Initialize RC input */
void RC_Init(RC_Handle_t *hrc)
{
    memset(hrc, 0, sizeof(RC_Handle_t));
    
    /* Set default pulse widths to center */
    for (int i = 0; i < RC_NUM_CHANNELS; i++) {
        hrc->pulse_us[i] = RC_MID_PULSE_US;
        hrc->filtered_pulse_us[i] = RC_MID_PULSE_US;
        hrc->stable_pulse_us[i] = RC_MID_PULSE_US;
    }
    hrc->pulse_us[RC_CH_THROTTLE] = RC_MIN_PULSE_US;  // Throttle starts at minimum
    hrc->filtered_pulse_us[RC_CH_THROTTLE] = RC_MIN_PULSE_US;
    hrc->stable_pulse_us[RC_CH_THROTTLE] = RC_MIN_PULSE_US;
    
    /* Initialize RC data */
    hrc->data.throttle = 0.0f;
    hrc->data.roll = 0.0f;
    hrc->data.pitch = 0.0f;
    hrc->data.yaw = 0.0f;
    hrc->data.valid = false;
    hrc->data.last_update = 0;

    /* Cortex-M cycle counter gives sub-microsecond edge timing. HAL_GetTick()
       is only 1 ms and cannot resolve normal 1000-2000 us RC pulses. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    /* Configure GPIO pins for EXTI (rising and falling edge) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = PIN_THROTTLE | PIN_ROLL | PIN_YAW;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PIN_PITCH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    /* Enable EXTI interrupts for the RC pins */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    
    HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static uint16_t RC_FilterPulse(RC_Handle_t *hrc, uint8_t channel)
{
    /* Update once per newly captured receiver pulse, not on every 100 Hz
       control iteration. Then move only in discrete RC_QUANTUM_US steps. */
    uint32_t sample_time = hrc->last_pulse_time[channel];
    if (sample_time != hrc->filtered_sample_time[channel]) {
        hrc->filtered_sample_time[channel] = sample_time;
        float raw = (float)hrc->pulse_us[channel];
        hrc->filtered_pulse_us[channel] += RC_FILTER_ALPHA *
            (raw - hrc->filtered_pulse_us[channel]);

        float difference = hrc->filtered_pulse_us[channel] -
                           (float)hrc->stable_pulse_us[channel];
        if (difference >= (float)RC_QUANTUM_US) {
            uint16_t steps = (uint16_t)(difference / (float)RC_QUANTUM_US);
            hrc->stable_pulse_us[channel] += steps * RC_QUANTUM_US;
        } else if (difference <= -(float)RC_QUANTUM_US) {
            uint16_t steps = (uint16_t)((-difference) / (float)RC_QUANTUM_US);
            hrc->stable_pulse_us[channel] -= steps * RC_QUANTUM_US;
        }
    }
    return hrc->stable_pulse_us[channel];
}

/* GPIO EXTI callback handler */
void RC_EXTI_Handler(RC_Handle_t *hrc, uint16_t GPIO_Pin)
{
    int8_t channel = RC_PinToChannel(GPIO_Pin);
    if (channel < 0) return;
    
    uint32_t now = HAL_GetTick();
    uint32_t now_cycles = DWT->CYCCNT;
    GPIO_PinState state = HAL_GPIO_ReadPin(RC_ChannelPort((uint8_t)channel),
                                          GPIO_Pin);

    hrc->edge_count[channel]++;
    hrc->last_edge_time[channel] = now;
    
    if (state == GPIO_PIN_SET) {
        /* Rising edge - start measuring */
        hrc->rise_time[channel] = now_cycles;
        hrc->measuring[channel] = true;
    } else if (hrc->measuring[channel]) {
        /* Falling edge - calculate pulse width */
        uint32_t elapsed_cycles = now_cycles - hrc->rise_time[channel];
        uint32_t cycles_per_us = SystemCoreClock / 1000000U;
        uint32_t pulse = elapsed_cycles / cycles_per_us;
        hrc->measuring[channel] = false;
        
        /* Accept a little margin around normal 1000-2000 us RC PWM. */
        if (pulse >= 800U && pulse <= 2200U) {
            hrc->pulse_us[channel] = (uint16_t)pulse;
            hrc->last_pulse_time[channel] = now;
            hrc->pulse_count[channel]++;
        } else {
            hrc->rejected_count[channel]++;
        }
    }
}

/* Update RC input values */
void RC_Update(RC_Handle_t *hrc)
{
    uint32_t now = HAL_GetTick();
    bool all_valid = true;
    
    /* All proportional RC channels must have a recent captured pulse. */
    static const uint8_t captured_channels[] = {
        RC_CH_THROTTLE, RC_CH_ROLL, RC_CH_PITCH, RC_CH_YAW
    };
    for (uint8_t i = 0; i < sizeof(captured_channels); i++) {
        uint8_t channel = captured_channels[i];
        if (hrc->pulse_count[channel] == 0U ||
            now - hrc->last_pulse_time[channel] > RC_SIGNAL_TIMEOUT) {
            all_valid = false;
        }
    }
    
    /* Convert raw pulse widths to normalized values */
    
    /* Throttle: 0.0 to 1.0 */
    uint16_t throttle_us = RC_FilterPulse(hrc, RC_CH_THROTTLE);
    if (throttle_us < RC_MIN_PULSE_US) throttle_us = RC_MIN_PULSE_US;
    if (throttle_us > RC_MAX_PULSE_US) throttle_us = RC_MAX_PULSE_US;
    hrc->data.throttle = (float)(throttle_us - RC_MIN_PULSE_US) / 
                         (float)(RC_MAX_PULSE_US - RC_MIN_PULSE_US);
    
    /* Roll: -1.0 to 1.0 (will be converted to angle for PID) */
    uint16_t roll_us = RC_FilterPulse(hrc, RC_CH_ROLL);
    if (roll_us < RC_MIN_PULSE_US) roll_us = RC_MIN_PULSE_US;
    if (roll_us > RC_MAX_PULSE_US) roll_us = RC_MAX_PULSE_US;
    hrc->data.roll = (float)(roll_us - RC_MID_PULSE_US) / 
                     (float)(RC_MAX_PULSE_US - RC_MID_PULSE_US);
    
    /* Pitch: -1.0 to 1.0 (will be converted to angle for PID) */
    uint16_t pitch_us = RC_FilterPulse(hrc, RC_CH_PITCH);
    if (pitch_us < RC_MIN_PULSE_US) pitch_us = RC_MIN_PULSE_US;
    if (pitch_us > RC_MAX_PULSE_US) pitch_us = RC_MAX_PULSE_US;
    hrc->data.pitch = (float)(pitch_us - RC_MID_PULSE_US) / 
                      (float)(RC_MAX_PULSE_US - RC_MID_PULSE_US);
    
    /* Yaw: -1.0 to 1.0 (front wheel steering passthrough) */
    uint16_t yaw_us = RC_FilterPulse(hrc, RC_CH_YAW);
    if (yaw_us < RC_MIN_PULSE_US) yaw_us = RC_MIN_PULSE_US;
    if (yaw_us > RC_MAX_PULSE_US) yaw_us = RC_MAX_PULSE_US;
    hrc->data.yaw = (float)(yaw_us - RC_MID_PULSE_US) / 
                    (float)(RC_MAX_PULSE_US - RC_MID_PULSE_US);
    
    /* Apply deadband to centered sticks */
    if (fabsf(hrc->data.roll) < 0.05f) hrc->data.roll = 0.0f;
    if (fabsf(hrc->data.pitch) < 0.05f) hrc->data.pitch = 0.0f;
    if (fabsf(hrc->data.yaw) < 0.05f) hrc->data.yaw = 0.0f;
    
    hrc->data.valid = all_valid;
    hrc->data.last_update = now;
}

/* Get RC input data */
void RC_GetInput(RC_Handle_t *hrc, RC_Input_t *input)
{
    *input = hrc->data;
}

/* Check if RC signal is valid */
bool RC_IsValid(RC_Handle_t *hrc)
{
    return hrc->data.valid;
}

bool RC_IsChannelValid(const RC_Handle_t *hrc, uint8_t channel)
{
    if (hrc == NULL || channel >= RC_NUM_CHANNELS)
        return false;
    uint32_t now = HAL_GetTick();
    return hrc->pulse_count[channel] != 0U &&
           now - hrc->last_pulse_time[channel] <= RC_SIGNAL_TIMEOUT;
}

bool RC_GetChannelDiagnostics(const RC_Handle_t *hrc, uint8_t channel,
                              RC_ChannelDiagnostics_t *diagnostics)
{
    if (hrc == NULL || diagnostics == NULL || channel >= RC_NUM_CHANNELS) {
        return false;
    }

    /* The ISR can update these fields between task instructions. Individual
       loads are atomic on Cortex-M4; copy each once to make a stable report. */
    uint32_t now = HAL_GetTick();
    uint32_t count = hrc->pulse_count[channel];
    uint32_t edge_count = hrc->edge_count[channel];
    uint32_t last_pulse = hrc->last_pulse_time[channel];
    uint32_t last_edge = hrc->last_edge_time[channel];

    diagnostics->pulse_us = hrc->pulse_us[channel];
    diagnostics->edge_count = edge_count;
    diagnostics->pulse_count = count;
    diagnostics->rejected_count = hrc->rejected_count[channel];
    diagnostics->age_ms = (count == 0U) ? now : (now - last_pulse);
    diagnostics->edge_age_ms = (edge_count == 0U) ? now : (now - last_edge);
    diagnostics->signal_present =
        (count != 0U) && (diagnostics->age_ms <= RC_SIGNAL_TIMEOUT);

    uint16_t pin = (channel == RC_CH_THROTTLE) ? PIN_THROTTLE :
                   (channel == RC_CH_ROLL) ? PIN_ROLL :
                   (channel == RC_CH_PITCH) ? PIN_PITCH : PIN_YAW;
    diagnostics->pin_high =
        (HAL_GPIO_ReadPin(RC_ChannelPort(channel), pin) == GPIO_PIN_SET);
    return true;
}

/* Convert normalized stick position to angle (radians) */
float RC_StickToAngle(float stick, float max_angle_rad)
{
    return stick * max_angle_rad;
}
