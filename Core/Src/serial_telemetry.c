/**
 ******************************************************************************
 * @file    serial_telemetry.c
 * @brief   Serial debug telemetry output for flight data
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "serial_telemetry.h"
#include "main.h"
#include <stdio.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define RAD_TO_DEG(x) ((x) * 57.2957795f)

/* External variables --------------------------------------------------------*/
extern FlightPID_t flight_pid;

/* Functions -----------------------------------------------------------------*/

/**
 * @brief Initialize serial telemetry
 */
void SerialTelemetry_Init(SerialTelemetry_Handle_t *handle, uint8_t rate_hz)
{
    if (rate_hz < 1) rate_hz = 1;
    if (rate_hz > 50) rate_hz = 50;
    
    handle->print_interval_ms = 1000 / rate_hz;
    handle->last_print_time = 0;
    handle->enabled = true;
}

/**
 * @brief Check if ready to print
 */
bool SerialTelemetry_ReadyToPrint(SerialTelemetry_Handle_t *handle)
{
    if (!handle->enabled) return false;
    
    uint32_t now = HAL_GetTick();
    if ((now - handle->last_print_time) >= handle->print_interval_ms)
    {
        handle->last_print_time = now;
        return true;
    }
    return false;
}

/**
 * @brief Enable/disable serial telemetry
 */
void SerialTelemetry_SetEnabled(SerialTelemetry_Handle_t *handle, bool enabled)
{
    handle->enabled = enabled;
}

/**
 * @brief Print all telemetry data
 */
void SerialTelemetry_Print(SerialTelemetry_Handle_t *handle, const FlightState_t *state,
                          float servo_roll, float servo_pitch, float servo_yaw, float throttle)
{
    if (!handle->enabled) return;
    
    // Header with timestamp
    printf("\r\n");
    printf("========================================\n");
    printf("       FLIGHT TELEMETRY DATA\n");
    printf("========================================\n");
    printf("Time: %lu ms | Loop: %lu\n\n", HAL_GetTick(), state->loop_count);
    
    // GPS Data
    printf("--- GPS ---\n");
    if (state->gps.fix_valid)
    {
        printf("Position: %.6f%c, %.6f%c\n", 
               fabsf(state->gps.latitude), 
               state->gps.latitude >= 0 ? 'N' : 'S',
               fabsf(state->gps.longitude),
               state->gps.longitude >= 0 ? 'E' : 'W');
        printf("Altitude: %.1f m | Speed: %.1f m/s\n", 
               state->gps.altitude, state->gps.speed);
        printf("Course: %.1f deg | Satellites: %d\n", 
               state->gps.course, state->gps.satellites);
    }
    else
    {
        printf("GPS: NO FIX | Satellites: %d\n", state->gps.satellites);
    }
    printf("\n");
    
    // IMU Data
    printf("--- IMU (MPU6050) ---\n");
    if (state->imu.valid)
    {
        printf("Gyro:  X: %+6.2f  Y: %+6.2f  Z: %+6.2f rad/s\n",
               state->imu.gyro.x, state->imu.gyro.y, state->imu.gyro.z);
        printf("Accel: X: %+6.2f  Y: %+6.2f  Z: %+6.2f m/s²\n",
               state->imu.accel.x, state->imu.accel.y, state->imu.accel.z);
        printf("Temperature: %.1f °C\n", state->imu.temperature);
    }
    else
    {
        printf("IMU: NOT VALID\n");
    }
    printf("\n");
    
    // Barometer Data
    printf("--- Barometer (MS5611) ---\n");
    if (state->baro.valid)
    {
        printf("Pressure: %.0f Pa | Temperature: %.1f °C\n",
               state->baro.pressure, state->baro.temperature);
        printf("Altitude: %.1f m (baro)\n", state->baro.altitude);
    }
    else
    {
        printf("Barometer: NOT VALID\n");
    }
    printf("\n");
    
    // Orientation (from Kalman filter)
    printf("--- Orientation (Kalman Filtered) ---\n");
    printf("Roll:  %+7.2f deg\n", RAD_TO_DEG(state->orientation.roll));
    printf("Pitch: %+7.2f deg\n", RAD_TO_DEG(state->orientation.pitch));
    printf("Yaw:   %+7.2f deg\n", RAD_TO_DEG(state->orientation.yaw));
    printf("\n");
    
    // PID Targets and Outputs
    printf("--- PID Control ---\n");
    printf("Targets:  Roll: %+7.2f° | Pitch: %+7.2f° | Yaw: %+7.2f°\n",
           RAD_TO_DEG(state->setpoint.roll),
           RAD_TO_DEG(state->setpoint.pitch),
           RAD_TO_DEG(state->setpoint.yaw));
    printf("Outputs:  Roll: %+6.3f  | Pitch: %+6.3f  | Yaw: %+6.3f\n",
           flight_pid.roll_output,
           flight_pid.pitch_output,
           flight_pid.yaw_output);
    printf("\n");
    
    // Servo/ESC Outputs
    printf("--- Actuators ---\n");
    uint8_t pwm_roll = (uint8_t)((servo_roll + 1.0f) * 127.5f);
    uint8_t pwm_pitch = (uint8_t)((servo_pitch + 1.0f) * 127.5f);
    uint8_t pwm_yaw = (uint8_t)((servo_yaw + 1.0f) * 127.5f);
    uint8_t pwm_throttle = (uint8_t)(throttle * 255.0f);
    
    printf("Servos: Roll: %3d | Pitch: %3d | Yaw: %3d (0-255)\n",
           pwm_roll, pwm_pitch, pwm_yaw);
    printf("Throttle: %3d (0-255) | %.1f%%\n", pwm_throttle, throttle * 100.0f);
    printf("\n");
    
    // Power Data
    printf("--- Power ---\n");
    if (state->power.valid)
    {
        printf("Voltage: %.2f V | Current: %.2f A\n",
               state->power.voltage, state->power.current);
        printf("Power: %.1f W\n", state->power.power);
    }
    else
    {
        printf("Power sensor: NOT VALID\n");
    }
    printf("\n");
    
    // System Health
    printf("--- System Health ---\n");
    uint32_t now = HAL_GetTick();
    printf("Last IMU update: %lu ms ago\n", now - state->last_imu_update);
    printf("Last GPS update: %lu ms ago\n", now - state->last_gps_update);
    printf("Last Baro update: %lu ms ago\n", now - state->last_baro_update);
    printf("========================================\n");
    printf("\r\n");
}
