/**
 * @file kalman.c
 * @brief Kalman filter implementation for orientation estimation
 */
#include "kalman.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Initialize single axis Kalman filter */
static void Kalman1D_Init(Kalman1D_t *kf)
{
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->rate = 0.0f;
    
    // Initial error covariance
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
    
    // Default tuning parameters
    kf->Q_angle = 0.001f;    // Process noise - angle
    kf->Q_bias = 0.003f;     // Process noise - bias
    kf->R_measure = 0.03f;   // Measurement noise
}

/* Initialize full Kalman filter */
void Kalman_Init(KalmanFilter_t *kf)
{
    Kalman1D_Init(&kf->roll);
    Kalman1D_Init(&kf->pitch);
    
    kf->yaw = 0.0f;
    kf->yaw_rate = 0.0f;
    kf->dt = 0.01f;  // Default 100Hz
    kf->last_update = 0;
}

/* Set tuning parameters */
void Kalman_SetTuning(KalmanFilter_t *kf, float q_angle, float q_bias, float r_measure)
{
    kf->roll.Q_angle = q_angle;
    kf->roll.Q_bias = q_bias;
    kf->roll.R_measure = r_measure;
    
    kf->pitch.Q_angle = q_angle;
    kf->pitch.Q_bias = q_bias;
    kf->pitch.R_measure = r_measure;
}

/* Single axis Kalman filter update */
float Kalman1D_Update(Kalman1D_t *kf, float accel_angle, float gyro_rate, float dt)
{
    /*
     * Step 1: Predict
     * x_k|k-1 = A * x_k-1|k-1 + B * u_k
     * 
     * State vector: [angle, bias]'
     * A = [1, -dt; 0, 1]
     * B = [dt; 0]
     * u = gyro_rate
     */
    
    // Unbiased rate
    kf->rate = gyro_rate - kf->bias;
    
    // Predicted angle
    kf->angle += dt * kf->rate;
    
    /*
     * Step 2: Update error covariance
     * P_k|k-1 = A * P_k-1|k-1 * A' + Q
     */
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;
    
    /*
     * Step 3: Calculate Kalman gain
     * K = P_k|k-1 * H' * (H * P_k|k-1 * H' + R)^-1
     * H = [1, 0]
     */
    float S = kf->P[0][0] + kf->R_measure;  // Innovation covariance
    
    float K[2];  // Kalman gain
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    
    /*
     * Step 4: Update estimate with measurement
     * x_k|k = x_k|k-1 + K * (z_k - H * x_k|k-1)
     */
    float y = accel_angle - kf->angle;  // Innovation (measurement residual)
    
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;
    
    /*
     * Step 5: Update error covariance
     * P_k|k = (I - K * H) * P_k|k-1
     */
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;
    
    return kf->angle;
}

/* Update Kalman filter with new sensor data */
void Kalman_Update(KalmanFilter_t *kf, const Vector3f_t *accel, const Vector3f_t *gyro, float dt)
{
    kf->dt = dt;
    
    /*
     * Calculate roll and pitch from accelerometer
     * Roll: rotation around X axis
     * Pitch: rotation around Y axis
     * 
     * These are valid when the device is not accelerating significantly
     */
    
    // Normalize accelerometer vector magnitude check
    float accel_mag = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    
    // Only use accelerometer if magnitude is close to 1g (9.81 m/s^2)
    // This helps reject accelerometer data during high-g maneuvers
    bool accel_valid = (accel_mag > 7.0f && accel_mag < 13.0f);
    
    float accel_roll = 0.0f;
    float accel_pitch = 0.0f;
    
    if (accel_valid) {
        // Calculate roll from accelerometer
        // roll = atan2(ay, az)
        accel_roll = atan2f(accel->y, accel->z);
        
        // Calculate pitch from accelerometer
        // pitch = atan2(-ax, sqrt(ay^2 + az^2))
        accel_pitch = atan2f(-accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z));
    } else {
        // Use current estimate as "measurement" during high-g maneuvers
        accel_roll = kf->roll.angle;
        accel_pitch = kf->pitch.angle;
    }
    
    // Update roll and pitch with Kalman filter
    Kalman1D_Update(&kf->roll, accel_roll, gyro->x, dt);
    Kalman1D_Update(&kf->pitch, accel_pitch, gyro->y, dt);
    
    // Yaw: Integrate gyroscope (no magnetometer)
    // Yaw will drift over time without absolute reference
    kf->yaw_rate = gyro->z;
    kf->yaw += gyro->z * dt;
    
    // Normalize yaw to [-PI, PI]
    while (kf->yaw > M_PI) kf->yaw -= 2.0f * M_PI;
    while (kf->yaw < -M_PI) kf->yaw += 2.0f * M_PI;
}

/* Get current orientation estimate */
void Kalman_GetOrientation(KalmanFilter_t *kf, Orientation_t *orientation)
{
    orientation->roll = kf->roll.angle;
    orientation->pitch = kf->pitch.angle;
    orientation->yaw = kf->yaw;
}
