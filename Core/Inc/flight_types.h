/**
 * @file flight_types.h
 * @brief Common type definitions for the flight controller
 */
#ifndef FLIGHT_TYPES_H
#define FLIGHT_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * SENSOR DATA STRUCTURES
 * ============================================================================ */

typedef struct {
    float x;
    float y;
    float z;
} Vector3f_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Vector3i_t;

typedef struct {
    Vector3f_t accel;      // m/s^2
    Vector3f_t gyro;       // rad/s
    float temperature;     // Celsius
    bool valid;
} IMU_Data_t;

typedef struct {
    float pressure;        // Pa
    float temperature;     // Celsius
    float altitude;        // meters (calculated)
    bool valid;
} Baro_Data_t;

typedef struct {
    float latitude;        // degrees
    float longitude;       // degrees
    float altitude;        // meters
    float speed;           // m/s
    float course;          // degrees
    uint8_t satellites;
    bool fix_valid;
} GPS_Data_t;

typedef struct {
    float voltage;         // Volts
    float current;         // Amps
    float power;           // Watts
    bool valid;
} Power_Data_t;

/* ============================================================================
 * ORIENTATION & CONTROL STRUCTURES
 * ============================================================================ */

typedef struct {
    float roll;            // radians
    float pitch;           // radians
    float yaw;             // radians
} Orientation_t;

typedef struct {
    float roll;            // radians
    float pitch;           // radians
    float yaw;             // radians
} SetPoint_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float integral_limit;
} PID_Controller_t;

/* ============================================================================
 * TELEMETRY STRUCTURES
 * ============================================================================ */

#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];     // 0xAA 0x55
    uint32_t timestamp;    // ms since boot
    
    // Orientation
    int16_t roll;          // deg * 100
    int16_t pitch;         // deg * 100
    int16_t yaw;           // deg * 100
    
    // GPS
    int32_t latitude;      // deg * 1e7
    int32_t longitude;     // deg * 1e7
    int16_t gps_altitude;  // meters
    uint8_t satellites;
    
    // Barometer
    int16_t baro_altitude; // meters
    
    // Power
    uint16_t voltage;      // mV
    uint16_t current;      // mA
    
    // Control outputs
    uint8_t servo_roll;    // 0-255
    uint8_t servo_pitch;   // 0-255
    uint8_t servo_yaw;     // 0-255
    uint8_t esc_throttle;  // 0-255
    
    uint8_t checksum;
} Telemetry_Packet_t;
#pragma pack(pop)

/* ============================================================================
 * SYSTEM STATE
 * ============================================================================ */

typedef struct {
    IMU_Data_t imu;
    Baro_Data_t baro;
    GPS_Data_t gps;
    Power_Data_t power;
    Orientation_t orientation;
    SetPoint_t setpoint;
    uint32_t loop_count;
    uint32_t last_imu_update;
    uint32_t last_gps_update;
    uint32_t last_baro_update;
} FlightState_t;

#endif /* FLIGHT_TYPES_H */
