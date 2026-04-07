/**
 ******************************************************************************
 * @file    serial_telemetry.h
 * @brief   Serial debug telemetry output for flight data
 ******************************************************************************
 */

#ifndef SERIAL_TELEMETRY_H
#define SERIAL_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "flight_types.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define SERIAL_TELEMETRY_ENABLED  1  // Set to 0 to disable

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint32_t print_interval_ms;      // Print interval in milliseconds
    uint32_t last_print_time;        // Last print timestamp
    bool enabled;                     // Enable/disable flag
} SerialTelemetry_Handle_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize serial telemetry
 * @param handle Pointer to telemetry handle
 * @param rate_hz Print rate in Hz (1-10 recommended)
 */
void SerialTelemetry_Init(SerialTelemetry_Handle_t *handle, uint8_t rate_hz);

/**
 * @brief Print all telemetry data (call at configured rate)
 * @param handle Pointer to telemetry handle
 * @param state Pointer to flight state structure
 * @param servo_roll Servo roll output (-1.0 to +1.0)
 * @param servo_pitch Servo pitch output (-1.0 to +1.0)
 * @param servo_yaw Servo yaw output (-1.0 to +1.0)
 * @param throttle Throttle command (0.0 to 1.0)
 */
void SerialTelemetry_Print(SerialTelemetry_Handle_t *handle, const FlightState_t *state,
                          float servo_roll, float servo_pitch, float servo_yaw, float throttle);

/**
 * @brief Check if ready to print (based on interval)
 * @param handle Pointer to telemetry handle
 * @retval true if ready to print, false otherwise
 */
bool SerialTelemetry_ReadyToPrint(SerialTelemetry_Handle_t *handle);

/**
 * @brief Enable/disable serial telemetry
 * @param handle Pointer to telemetry handle
 * @param enabled true to enable, false to disable
 */
void SerialTelemetry_SetEnabled(SerialTelemetry_Handle_t *handle, bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_TELEMETRY_H */
