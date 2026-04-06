/**
 * @file servo_esc.h
 * @brief Servo and ESC control via PWM
 */
#ifndef SERVO_ESC_H
#define SERVO_ESC_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>

/* Servo pulse widths (microseconds) */
#define SERVO_MIN_PULSE_US      1000   // -90 degrees
#define SERVO_MID_PULSE_US      1500   // 0 degrees (center)
#define SERVO_MAX_PULSE_US      2000   // +90 degrees

/* ESC pulse widths (microseconds) */
#define ESC_MIN_PULSE_US        1000   // 0% throttle
#define ESC_MAX_PULSE_US        2000   // 100% throttle
#define ESC_ARM_PULSE_US        1000   // Arming pulse

/* Skywalker 100A V2-UBEC specific timing */
#define ESC_CALIB_HIGH_US       2000   // Full throttle for calibration
#define ESC_CALIB_LOW_US        1000   // Zero throttle for calibration
#define ESC_PROG_ENTER_DELAY    2000   // ms to hold high throttle to enter programming
#define ESC_PROG_CONFIRM_DELAY  1000   // ms between programming beeps

/* Timer configuration */
// Timer frequency = 16MHz / (prescaler+1) = 16MHz / 200 = 80kHz
// PWM period = (ARR+1) / 80kHz = 1600 / 80000 = 20ms = 50Hz
#define SERVO_TIMER_FREQ        80000  // Hz after prescaler
#define SERVO_PWM_PERIOD        1600   // ARR+1 for 50Hz

/* Servo handle */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t min_pulse;    // Timer counts for min position
    uint16_t max_pulse;    // Timer counts for max position
    uint16_t current_pulse;
} Servo_Handle_t;

/* ESC handle */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t min_pulse;
    uint16_t max_pulse;
    bool armed;
    GPIO_TypeDef *prog_port;    // Programming mode trigger pin
    uint16_t prog_pin;
} ESC_Handle_t;

/* Skywalker ESC programming modes */
typedef enum {
    ESC_PROG_BRAKE_OFF = 1,
    ESC_PROG_BRAKE_ON,
    ESC_PROG_BATTERY_AUTO,
    ESC_PROG_BATTERY_LIPO,
    ESC_PROG_BATTERY_NIMH,
    ESC_PROG_CUTOFF_LOW,
    ESC_PROG_CUTOFF_MED,
    ESC_PROG_CUTOFF_HIGH,
    ESC_PROG_TIMING_LOW,
    ESC_PROG_TIMING_MED,
    ESC_PROG_TIMING_HIGH
} ESC_ProgOption_t;

/* Servo functions */
void Servo_Init(Servo_Handle_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetAngle(Servo_Handle_t *servo, float angle);  // -90 to +90 degrees
void Servo_SetPulse(Servo_Handle_t *servo, uint16_t pulse_us);
void Servo_SetNormalized(Servo_Handle_t *servo, float value);  // -1.0 to +1.0

/* ESC functions */
void ESC_Init(ESC_Handle_t *esc, TIM_HandleTypeDef *htim, uint32_t channel);
void ESC_InitWithProg(ESC_Handle_t *esc, TIM_HandleTypeDef *htim, uint32_t channel,
                      GPIO_TypeDef *prog_port, uint16_t prog_pin);
void ESC_Arm(ESC_Handle_t *esc);
void ESC_Disarm(ESC_Handle_t *esc);
void ESC_SetThrottle(ESC_Handle_t *esc, float throttle);  // 0.0 to 1.0
bool ESC_IsArmed(ESC_Handle_t *esc);

/* Skywalker 100A V2-UBEC Calibration/Programming */
void ESC_CalibrateThrottle(ESC_Handle_t *esc);  // Throttle range calibration
void ESC_EnterProgramMode(ESC_Handle_t *esc);   // Enter programming mode
void ESC_ExitProgramMode(ESC_Handle_t *esc);    // Exit and save
void ESC_ProgSelectOption(ESC_Handle_t *esc, uint8_t beeps);  // Select by beep count
void ESC_TriggerProgPin(ESC_Handle_t *esc);     // Pulse PC10 for programming confirm

/* Helper: Convert microseconds to timer counts */
uint16_t PWM_UsToTicks(uint16_t us);

#endif /* SERVO_ESC_H */
