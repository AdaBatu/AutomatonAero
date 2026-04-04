/**
 * @file servo_esc.c
 * @brief Servo and ESC PWM control implementation
 */
#include "servo_esc.h"

/* Convert microseconds to timer ticks */
uint16_t PWM_UsToTicks(uint16_t us)
{
    // Timer frequency after prescaler = 16MHz / 200 = 80kHz
    // So 1 tick = 12.5us
    // ticks = us / 12.5 = us * 80 / 1000 = us * 0.08
    return (uint16_t)((uint32_t)us * SERVO_TIMER_FREQ / 1000000);
}

/* ============================================================================
 * SERVO FUNCTIONS
 * ============================================================================ */

/* Initialize servo */
void Servo_Init(Servo_Handle_t *servo, TIM_HandleTypeDef *htim, uint32_t channel)
{
    servo->htim = htim;
    servo->channel = channel;
    servo->min_pulse = PWM_UsToTicks(SERVO_MIN_PULSE_US);
    servo->max_pulse = PWM_UsToTicks(SERVO_MAX_PULSE_US);
    servo->current_pulse = PWM_UsToTicks(SERVO_MID_PULSE_US);
    
    // Start PWM
    HAL_TIM_PWM_Start(htim, channel);
    
    // Set to center position
    __HAL_TIM_SET_COMPARE(htim, channel, servo->current_pulse);
}

/* Set servo angle (-90 to +90 degrees) */
void Servo_SetAngle(Servo_Handle_t *servo, float angle)
{
    // Clamp angle
    if (angle < -90.0f) angle = -90.0f;
    if (angle > 90.0f) angle = 90.0f;
    
    // Map angle to pulse width
    // -90° -> min_pulse, 0° -> mid, +90° -> max_pulse
    uint16_t mid_pulse = PWM_UsToTicks(SERVO_MID_PULSE_US);
    uint16_t range = servo->max_pulse - servo->min_pulse;
    
    // angle/90 gives -1 to +1, then scale to pulse range
    float normalized = angle / 90.0f;
    uint16_t pulse = mid_pulse + (int16_t)(normalized * (range / 2));
    
    servo->current_pulse = pulse;
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse);
}

/* Set servo pulse width in microseconds */
void Servo_SetPulse(Servo_Handle_t *servo, uint16_t pulse_us)
{
    // Clamp pulse
    if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;
    
    uint16_t ticks = PWM_UsToTicks(pulse_us);
    servo->current_pulse = ticks;
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, ticks);
}

/* Set servo position from normalized value (-1.0 to +1.0) */
void Servo_SetNormalized(Servo_Handle_t *servo, float value)
{
    // Clamp value
    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f) value = 1.0f;
    
    // Map to pulse range
    uint16_t mid_pulse = PWM_UsToTicks(SERVO_MID_PULSE_US);
    uint16_t half_range = (servo->max_pulse - servo->min_pulse) / 2;
    
    uint16_t pulse = mid_pulse + (int16_t)(value * half_range);
    
    servo->current_pulse = pulse;
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse);
}

/* ============================================================================
 * ESC FUNCTIONS
 * ============================================================================ */

/* Initialize ESC */
void ESC_Init(ESC_Handle_t *esc, TIM_HandleTypeDef *htim, uint32_t channel)
{
    esc->htim = htim;
    esc->channel = channel;
    esc->min_pulse = PWM_UsToTicks(ESC_MIN_PULSE_US);
    esc->max_pulse = PWM_UsToTicks(ESC_MAX_PULSE_US);
    esc->armed = false;
    esc->prog_port = NULL;
    esc->prog_pin = 0;
    
    // Start PWM at minimum throttle
    HAL_TIM_PWM_Start(htim, channel);
    __HAL_TIM_SET_COMPARE(htim, channel, esc->min_pulse);
}

/* Initialize ESC with programming pin (PC10 for Skywalker) */
void ESC_InitWithProg(ESC_Handle_t *esc, TIM_HandleTypeDef *htim, uint32_t channel,
                      GPIO_TypeDef *prog_port, uint16_t prog_pin)
{
    ESC_Init(esc, htim, channel);
    esc->prog_port = prog_port;
    esc->prog_pin = prog_pin;
}

/* Arm ESC (send minimum throttle for a period) */
void ESC_Arm(ESC_Handle_t *esc)
{
    // Set to minimum throttle (arming signal)
    __HAL_TIM_SET_COMPARE(esc->htim, esc->channel, esc->min_pulse);
    
    // ESC typically needs 2-3 seconds at minimum throttle to arm
    // This function just sets the signal - caller should wait
    esc->armed = true;
}

/* Disarm ESC */
void ESC_Disarm(ESC_Handle_t *esc)
{
    // Set to minimum throttle
    __HAL_TIM_SET_COMPARE(esc->htim, esc->channel, esc->min_pulse);
    esc->armed = false;
}

/* Set ESC throttle (0.0 to 1.0) */
void ESC_SetThrottle(ESC_Handle_t *esc, float throttle)
{
    if (!esc->armed) {
        // Don't allow throttle if not armed
        return;
    }
    
    // Clamp throttle
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;
    
    // Map to pulse range
    uint16_t pulse_range = esc->max_pulse - esc->min_pulse;
    uint16_t pulse = esc->min_pulse + (uint16_t)(throttle * pulse_range);
    
    __HAL_TIM_SET_COMPARE(esc->htim, esc->channel, pulse);
}

/* Check if ESC is armed */
bool ESC_IsArmed(ESC_Handle_t *esc)
{
    return esc->armed;
}

/* ============================================================================
 * SKYWALKER 100A V2-UBEC ESC PROGRAMMING FUNCTIONS
 * ============================================================================ */

/**
 * @brief Calibrate ESC throttle range
 * @note  Connect battery with throttle at max, then go to min
 *        ESC will beep to confirm high/low points
 * 
 * Procedure:
 * 1. Disconnect battery
 * 2. Set throttle to maximum (2000us)
 * 3. Connect battery - ESC beeps
 * 4. Set throttle to minimum (1000us) - ESC beeps to confirm
 */
void ESC_CalibrateThrottle(ESC_Handle_t *esc)
{
    // Step 1: Set to maximum throttle (user should connect battery now)
    uint16_t max_ticks = PWM_UsToTicks(ESC_CALIB_HIGH_US);
    __HAL_TIM_SET_COMPARE(esc->htim, esc->channel, max_ticks);
    
    // Wait for user to connect battery (ESC will beep)
    HAL_Delay(ESC_PROG_ENTER_DELAY);
    
    // Step 2: Set to minimum throttle
    uint16_t min_ticks = PWM_UsToTicks(ESC_CALIB_LOW_US);
    __HAL_TIM_SET_COMPARE(esc->htim, esc->channel, min_ticks);
    
    // ESC will confirm with beeps
    HAL_Delay(2000);
    
    esc->armed = false;
}

/**
 * @brief Enter ESC programming mode
 * @note  Skywalker ESC: Hold full throttle for 2+ seconds after power-on
 */
void ESC_EnterProgramMode(ESC_Handle_t *esc)
{
    // Set maximum throttle
    uint16_t max_ticks = PWM_UsToTicks(ESC_CALIB_HIGH_US);
    __HAL_TIM_SET_COMPARE(esc->htim, esc->channel, max_ticks);
    
    // Hold for programming mode entry
    HAL_Delay(ESC_PROG_ENTER_DELAY);
    
    // ESC enters programming mode and starts beeping menu options
}

/**
 * @brief Exit programming mode and save settings
 */
void ESC_ExitProgramMode(ESC_Handle_t *esc)
{
    // Return to minimum throttle to exit
    uint16_t min_ticks = PWM_UsToTicks(ESC_CALIB_LOW_US);
    __HAL_TIM_SET_COMPARE(esc->htim, esc->channel, min_ticks);
    
    HAL_Delay(1000);
    esc->armed = false;
}

/**
 * @brief Select programming option by beep count
 * @param beeps Number of beeps to wait for before confirming
 * @note  Skywalker programming: listen for beeps, trigger at desired option
 * 
 * Menu structure (typical):
 * 1 beep  - Brake (off/on)
 * 2 beeps - Battery type (auto/LiPo/NiMH)
 * 3 beeps - Low voltage cutoff (low/med/high)
 * 4 beeps - Timing (low/med/high)
 */
void ESC_ProgSelectOption(ESC_Handle_t *esc, uint8_t beeps)
{
    // Wait for the specified number of beeps
    // Each beep cycle is approximately 1 second apart
    HAL_Delay(beeps * ESC_PROG_CONFIRM_DELAY);
    
    // Trigger selection by briefly going to min throttle
    uint16_t min_ticks = PWM_UsToTicks(ESC_CALIB_LOW_US);
    __HAL_TIM_SET_COMPARE(esc->htim, esc->channel, min_ticks);
    HAL_Delay(200);
    
    // Return to max to continue in programming mode
    uint16_t max_ticks = PWM_UsToTicks(ESC_CALIB_HIGH_US);
    __HAL_TIM_SET_COMPARE(esc->htim, esc->channel, max_ticks);
}

/**
 * @brief Pulse the programming pin (PC10) for external trigger
 * @note  Can be used for ESCs that support external programming buttons
 */
void ESC_TriggerProgPin(ESC_Handle_t *esc)
{
    if (esc->prog_port == NULL) return;
    
    // Pulse the programming pin
    HAL_GPIO_WritePin(esc->prog_port, esc->prog_pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(esc->prog_port, esc->prog_pin, GPIO_PIN_RESET);
}
