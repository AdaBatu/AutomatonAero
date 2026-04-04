/**
 * @file pid.h
 * @brief PID controller for servo actuation
 */
#ifndef PID_H
#define PID_H

#include "flight_types.h"

/* PID controller initialization */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd);

/* Set output limits */
void PID_SetLimits(PID_Controller_t *pid, float min, float max);

/* Set integral windup limit */
void PID_SetIntegralLimit(PID_Controller_t *pid, float limit);

/* Compute PID output */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement, float dt);

/* Reset integral and derivative terms */
void PID_Reset(PID_Controller_t *pid);

/* Update tuning parameters */
void PID_SetTuning(PID_Controller_t *pid, float Kp, float Ki, float Kd);

/* Flight controller with 3 PID controllers */
typedef struct {
    PID_Controller_t roll;
    PID_Controller_t pitch;
    PID_Controller_t yaw;
    
    SetPoint_t setpoint;
    
    // Output values (servo positions)
    float roll_output;
    float pitch_output;
    float yaw_output;
} FlightPID_t;

/* Flight PID functions */
void FlightPID_Init(FlightPID_t *fpid);
void FlightPID_SetSetpoint(FlightPID_t *fpid, float roll, float pitch, float yaw);
void FlightPID_Update(FlightPID_t *fpid, const Orientation_t *orientation, float dt);
void FlightPID_GetOutputs(FlightPID_t *fpid, float *roll, float *pitch, float *yaw);

#endif /* PID_H */
