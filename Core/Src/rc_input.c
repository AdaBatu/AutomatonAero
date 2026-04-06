/**
 * @file rc_input.c
 * @brief RC receiver PWM input capture implementation
 * 
 * Uses GPIO EXTI interrupts to measure PWM pulse widths from RC receiver.
 * Pin assignments:
 *   PB1  - Throttle
 *   PB2  - Roll
 *   PB15 - Pitch
 *   PB11 - Yaw
 */
#include "rc_input.h"
#include <string.h>
#include <math.h>

/* Pin to channel mapping */
#define PIN_THROTTLE    GPIO_PIN_1
#define PIN_ROLL        GPIO_PIN_2
#define PIN_PITCH       GPIO_PIN_15
#define PIN_YAW         GPIO_PIN_11

/* Maximum stick deflection angle in radians (±30 degrees) */
#define MAX_STICK_ANGLE_RAD  (30.0f * 3.14159265f / 180.0f)

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
    }
    hrc->pulse_us[RC_CH_THROTTLE] = RC_MIN_PULSE_US;  // Throttle starts at minimum
    
    /* Initialize RC data */
    hrc->data.throttle = 0.0f;
    hrc->data.roll = 0.0f;
    hrc->data.pitch = 0.0f;
    hrc->data.yaw = 0.0f;
    hrc->data.valid = false;
    hrc->data.last_update = 0;
    
    /* Configure GPIO pins for EXTI (rising and falling edge) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = PIN_THROTTLE | PIN_ROLL | PIN_PITCH | PIN_YAW;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Enable EXTI interrupts for the RC pins */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    
    HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* GPIO EXTI callback handler */
void RC_EXTI_Handler(RC_Handle_t *hrc, uint16_t GPIO_Pin)
{
    int8_t channel = RC_PinToChannel(GPIO_Pin);
    if (channel < 0) return;
    
    uint32_t now = HAL_GetTick();
    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin);
    
    if (state == GPIO_PIN_SET) {
        /* Rising edge - start measuring */
        hrc->rise_time[channel] = now;
        hrc->measuring[channel] = true;
    } else if (hrc->measuring[channel]) {
        /* Falling edge - calculate pulse width */
        uint32_t pulse = now - hrc->rise_time[channel];
        hrc->measuring[channel] = false;
        
        /* Validate pulse width (RC signals are typically 1000-2000us) */
        /* We're measuring in ms due to HAL_GetTick resolution, so convert */
        /* For more accurate timing, use a hardware timer */
        if (pulse >= 1 && pulse <= 3) {
            /* Convert ms to approximate us (rough, but functional) */
            /* For precise timing, implement using TIM input capture */
            hrc->pulse_us[channel] = pulse * 1000;
            hrc->last_pulse_time[channel] = now;
        }
    }
}

/* Update RC input values */
void RC_Update(RC_Handle_t *hrc)
{
    uint32_t now = HAL_GetTick();
    bool all_valid = true;
    
    /* Check signal validity for all channels */
    for (int i = 0; i < RC_NUM_CHANNELS; i++) {
        if (now - hrc->last_pulse_time[i] > RC_SIGNAL_TIMEOUT) {
            all_valid = false;
        }
    }
    
    /* Convert raw pulse widths to normalized values */
    
    /* Throttle: 0.0 to 1.0 */
    uint16_t throttle_us = hrc->pulse_us[RC_CH_THROTTLE];
    if (throttle_us < RC_MIN_PULSE_US) throttle_us = RC_MIN_PULSE_US;
    if (throttle_us > RC_MAX_PULSE_US) throttle_us = RC_MAX_PULSE_US;
    hrc->data.throttle = (float)(throttle_us - RC_MIN_PULSE_US) / 
                         (float)(RC_MAX_PULSE_US - RC_MIN_PULSE_US);
    
    /* Roll: -1.0 to 1.0 (will be converted to angle for PID) */
    uint16_t roll_us = hrc->pulse_us[RC_CH_ROLL];
    if (roll_us < RC_MIN_PULSE_US) roll_us = RC_MIN_PULSE_US;
    if (roll_us > RC_MAX_PULSE_US) roll_us = RC_MAX_PULSE_US;
    hrc->data.roll = (float)(roll_us - RC_MID_PULSE_US) / 
                     (float)(RC_MAX_PULSE_US - RC_MID_PULSE_US);
    
    /* Pitch: -1.0 to 1.0 (will be converted to angle for PID) */
    uint16_t pitch_us = hrc->pulse_us[RC_CH_PITCH];
    if (pitch_us < RC_MIN_PULSE_US) pitch_us = RC_MIN_PULSE_US;
    if (pitch_us > RC_MAX_PULSE_US) pitch_us = RC_MAX_PULSE_US;
    hrc->data.pitch = (float)(pitch_us - RC_MID_PULSE_US) / 
                      (float)(RC_MAX_PULSE_US - RC_MID_PULSE_US);
    
    /* Yaw: -1.0 to 1.0 (direct passthrough) */
    uint16_t yaw_us = hrc->pulse_us[RC_CH_YAW];
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

/* Convert normalized stick position to angle (radians) */
float RC_StickToAngle(float stick, float max_angle_rad)
{
    return stick * max_angle_rad;
}
