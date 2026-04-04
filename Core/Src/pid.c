/**
 * @file pid.c
 * @brief PID controller implementation
 */
#include "pid.h"

/* Initialize PID controller */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = -1.0f;
    pid->output_max = 1.0f;
    pid->integral_limit = 0.5f;
}

/* Set output limits */
void PID_SetLimits(PID_Controller_t *pid, float min, float max)
{
    pid->output_min = min;
    pid->output_max = max;
}

/* Set integral windup limit */
void PID_SetIntegralLimit(PID_Controller_t *pid, float limit)
{
    pid->integral_limit = limit;
}

/* Compute PID output */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement, float dt)
{
    // Calculate error
    float error = setpoint - measurement;
    
    // Proportional term
    float P = pid->Kp * error;
    
    // Integral term with anti-windup
    pid->integral += error * dt;
    
    // Clamp integral to prevent windup
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    float I = pid->Ki * pid->integral;
    
    // Derivative term (on error)
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->Kd * derivative;
    
    // Store error for next iteration
    pid->prev_error = error;
    
    // Calculate total output
    float output = P + I + D;
    
    // Clamp output
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    return output;
}

/* Reset PID controller state */
void PID_Reset(PID_Controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/* Update tuning parameters */
void PID_SetTuning(PID_Controller_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

/* ============================================================================
 * FLIGHT PID CONTROLLER (3-axis)
 * ============================================================================ */

/* Default PID gains - these should be tuned for specific aircraft */
#define DEFAULT_ROLL_KP     1.5f
#define DEFAULT_ROLL_KI     0.1f
#define DEFAULT_ROLL_KD     0.05f

#define DEFAULT_PITCH_KP    1.5f
#define DEFAULT_PITCH_KI    0.1f
#define DEFAULT_PITCH_KD    0.05f

#define DEFAULT_YAW_KP      2.0f
#define DEFAULT_YAW_KI      0.05f
#define DEFAULT_YAW_KD      0.1f

/* Initialize flight PID controller */
void FlightPID_Init(FlightPID_t *fpid)
{
    PID_Init(&fpid->roll, DEFAULT_ROLL_KP, DEFAULT_ROLL_KI, DEFAULT_ROLL_KD);
    PID_Init(&fpid->pitch, DEFAULT_PITCH_KP, DEFAULT_PITCH_KI, DEFAULT_PITCH_KD);
    PID_Init(&fpid->yaw, DEFAULT_YAW_KP, DEFAULT_YAW_KI, DEFAULT_YAW_KD);
    
    // Set output limits (normalized -1 to 1)
    PID_SetLimits(&fpid->roll, -1.0f, 1.0f);
    PID_SetLimits(&fpid->pitch, -1.0f, 1.0f);
    PID_SetLimits(&fpid->yaw, -1.0f, 1.0f);
    
    // Initialize setpoint to level flight
    fpid->setpoint.roll = 0.0f;
    fpid->setpoint.pitch = 0.0f;
    fpid->setpoint.yaw = 0.0f;
    
    // Initialize outputs
    fpid->roll_output = 0.0f;
    fpid->pitch_output = 0.0f;
    fpid->yaw_output = 0.0f;
}

/* Set desired orientation */
void FlightPID_SetSetpoint(FlightPID_t *fpid, float roll, float pitch, float yaw)
{
    fpid->setpoint.roll = roll;
    fpid->setpoint.pitch = pitch;
    fpid->setpoint.yaw = yaw;
}

/* Update PID controllers with current orientation */
void FlightPID_Update(FlightPID_t *fpid, const Orientation_t *orientation, float dt)
{
    // Compute PID outputs
    fpid->roll_output = PID_Compute(&fpid->roll, fpid->setpoint.roll, orientation->roll, dt);
    fpid->pitch_output = PID_Compute(&fpid->pitch, fpid->setpoint.pitch, orientation->pitch, dt);
    
    // For yaw, handle wrap-around at ±PI
    float yaw_error = fpid->setpoint.yaw - orientation->yaw;
    
    // Normalize yaw error to [-PI, PI]
    while (yaw_error > 3.14159265f) yaw_error -= 2.0f * 3.14159265f;
    while (yaw_error < -3.14159265f) yaw_error += 2.0f * 3.14159265f;
    
    // Compute yaw output using error directly
    fpid->yaw.prev_error = yaw_error;  // Store for next iteration
    fpid->yaw_output = PID_Compute(&fpid->yaw, fpid->setpoint.yaw, 
                                    fpid->setpoint.yaw - yaw_error, dt);
}

/* Get PID outputs */
void FlightPID_GetOutputs(FlightPID_t *fpid, float *roll, float *pitch, float *yaw)
{
    *roll = fpid->roll_output;
    *pitch = fpid->pitch_output;
    *yaw = fpid->yaw_output;
}
