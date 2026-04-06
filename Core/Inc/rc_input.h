/**
 * @file rc_input.h
 * @brief RC receiver PWM input capture
 * 
 * Pin assignments:
 *   PB1  - Throttle
 *   PB2  - Roll
 *   PB15 - Pitch
 *   PB11 - Yaw
 */
#ifndef RC_INPUT_H
#define RC_INPUT_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>

/* RC channel indices */
#define RC_CH_THROTTLE  0
#define RC_CH_ROLL      1
#define RC_CH_PITCH     2
#define RC_CH_YAW       3
#define RC_NUM_CHANNELS 4

/* RC pulse timing (microseconds) */
#define RC_MIN_PULSE_US     1000
#define RC_MID_PULSE_US     1500
#define RC_MAX_PULSE_US     2000
#define RC_PULSE_DEADBAND   50    // Ignore small changes around center

/* RC signal validity timeout (ms) */
#define RC_SIGNAL_TIMEOUT   500

/* RC input data */
typedef struct {
    float throttle;      // 0.0 to 1.0
    float roll;          // -1.0 to 1.0 (for PID setpoint, scaled to radians)
    float pitch;         // -1.0 to 1.0 (for PID setpoint, scaled to radians)
    float yaw;           // -1.0 to 1.0 (direct passthrough)
    bool valid;          // True if all channels have recent valid signals
    uint32_t last_update;
} RC_Input_t;

/* RC handle */
typedef struct {
    uint16_t pulse_us[RC_NUM_CHANNELS];   // Raw pulse widths
    uint32_t last_pulse_time[RC_NUM_CHANNELS];
    volatile uint32_t rise_time[RC_NUM_CHANNELS];
    volatile bool measuring[RC_NUM_CHANNELS];
    RC_Input_t data;
} RC_Handle_t;

/* Initialize RC input */
void RC_Init(RC_Handle_t *hrc);

/* Update RC input (call periodically) */
void RC_Update(RC_Handle_t *hrc);

/* Get RC input data */
void RC_GetInput(RC_Handle_t *hrc, RC_Input_t *input);

/* Check if RC signal is valid */
bool RC_IsValid(RC_Handle_t *hrc);

/* GPIO EXTI callback handler (call from HAL_GPIO_EXTI_Callback) */
void RC_EXTI_Handler(RC_Handle_t *hrc, uint16_t GPIO_Pin);

/* Convert normalized stick position to angle (radians) */
float RC_StickToAngle(float stick, float max_angle_rad);

#endif /* RC_INPUT_H */
