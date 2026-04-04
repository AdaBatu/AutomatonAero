/**
 * @file kalman.h
 * @brief Kalman filter for orientation estimation
 */
#ifndef KALMAN_H
#define KALMAN_H

#include "flight_types.h"

/* Kalman filter structure for single axis */
typedef struct {
    float angle;           // Estimated angle
    float bias;            // Estimated gyro bias
    float rate;            // Unbiased rate
    
    // Error covariance matrix
    float P[2][2];
    
    // Process noise
    float Q_angle;         // Process noise for angle
    float Q_bias;          // Process noise for bias
    
    // Measurement noise
    float R_measure;       // Measurement noise variance
} Kalman1D_t;

/* Full 3-axis Kalman filter */
typedef struct {
    Kalman1D_t roll;
    Kalman1D_t pitch;
    
    // Yaw handled separately (magnetometer or gyro integration)
    float yaw;
    float yaw_rate;
    
    // Timing
    float dt;
    uint32_t last_update;
} KalmanFilter_t;

/* Function prototypes */
void Kalman_Init(KalmanFilter_t *kf);
void Kalman_SetTuning(KalmanFilter_t *kf, float q_angle, float q_bias, float r_measure);

/* Update with accelerometer and gyroscope data */
void Kalman_Update(KalmanFilter_t *kf, 
                   const Vector3f_t *accel, 
                   const Vector3f_t *gyro, 
                   float dt);

/* Get current orientation estimate */
void Kalman_GetOrientation(KalmanFilter_t *kf, Orientation_t *orientation);

/* Single axis update (used internally) */
float Kalman1D_Update(Kalman1D_t *kf, float accel_angle, float gyro_rate, float dt);

#endif /* KALMAN_H */
