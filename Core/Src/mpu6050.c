/**
 * @file mpu6050.c
 * @brief MPU-6050 IMU driver implementation
 */
#include "mpu6050.h"
#include <math.h>

#define I2C_TIMEOUT     100

/* Internal helper functions */
static HAL_StatusTypeDef MPU6050_WriteReg(MPU6050_Handle_t *hdev, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hdev->hi2c, hdev->address, reg, I2C_MEMADD_SIZE_8BIT, 
                             &value, 1, I2C_TIMEOUT);
}

static HAL_StatusTypeDef MPU6050_ReadReg(MPU6050_Handle_t *hdev, uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(hdev->hi2c, hdev->address, reg, I2C_MEMADD_SIZE_8BIT, 
                            value, 1, I2C_TIMEOUT);
}

static HAL_StatusTypeDef MPU6050_ReadRegs(MPU6050_Handle_t *hdev, uint8_t reg, uint8_t *data, uint8_t len)
{
    return HAL_I2C_Mem_Read(hdev->hi2c, hdev->address, reg, I2C_MEMADD_SIZE_8BIT, 
                            data, len, I2C_TIMEOUT);
}

/* Initialize MPU-6050 */
HAL_StatusTypeDef MPU6050_Init(MPU6050_Handle_t *hdev, I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    
    hdev->hi2c = hi2c;
    hdev->address = MPU6050_ADDR;
    
    // Check device ID
    uint8_t who_am_i = MPU6050_WhoAmI(hdev);
    if (who_am_i != 0x68) {
        // Try alternate address
        hdev->address = MPU6050_ADDR_ALT;
        who_am_i = MPU6050_WhoAmI(hdev);
        if (who_am_i != 0x68) {
            return HAL_ERROR;
        }
    }
    
    // Wake up device (clear sleep bit)
    status = MPU6050_WriteReg(hdev, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (status != HAL_OK) return status;
    
    HAL_Delay(100);  // Wait for device to wake
    
    // Set clock source to PLL with X-axis gyro reference
    status = MPU6050_WriteReg(hdev, MPU6050_REG_PWR_MGMT_1, 0x01);
    if (status != HAL_OK) return status;
    
    // Configure sample rate divider (1kHz / (1+4) = 200Hz)
    status = MPU6050_WriteReg(hdev, MPU6050_REG_SMPLRT_DIV, 0x04);
    if (status != HAL_OK) return status;
    
    // Configure DLPF (Digital Low Pass Filter) - 44Hz bandwidth
    status = MPU6050_WriteReg(hdev, MPU6050_REG_CONFIG, MPU6050_DLPF_BW_42);
    if (status != HAL_OK) return status;
    
    // Configure gyroscope (±500°/s)
    status = MPU6050_WriteReg(hdev, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_500);
    if (status != HAL_OK) return status;
    hdev->gyro_scale = MPU6050_GYRO_SCALE_500;
    
    // Configure accelerometer (±4g)
    status = MPU6050_WriteReg(hdev, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_4G);
    if (status != HAL_OK) return status;
    hdev->accel_scale = MPU6050_ACCEL_SCALE_4G;
    
    // Initialize offsets to zero
    hdev->gyro_offset.x = 0.0f;
    hdev->gyro_offset.y = 0.0f;
    hdev->gyro_offset.z = 0.0f;
    hdev->accel_offset.x = 0.0f;
    hdev->accel_offset.y = 0.0f;
    hdev->accel_offset.z = 0.0f;
    
    return HAL_OK;
}

/* Read WHO_AM_I register */
uint8_t MPU6050_WhoAmI(MPU6050_Handle_t *hdev)
{
    uint8_t value = 0;
    MPU6050_ReadReg(hdev, MPU6050_REG_WHO_AM_I, &value);
    return value;
}

/* Read all sensor data */
HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_Handle_t *hdev, IMU_Data_t *data)
{
    uint8_t buffer[14];
    HAL_StatusTypeDef status;
    
    // Read all 14 bytes starting from ACCEL_XOUT_H
    status = MPU6050_ReadRegs(hdev, MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    if (status != HAL_OK) {
        data->valid = false;
        return status;
    }
    
    // Parse raw accelerometer data (big endian)
    int16_t accel_raw_x = (buffer[0] << 8) | buffer[1];
    int16_t accel_raw_y = (buffer[2] << 8) | buffer[3];
    int16_t accel_raw_z = (buffer[4] << 8) | buffer[5];
    
    // Parse raw temperature
    int16_t temp_raw = (buffer[6] << 8) | buffer[7];
    
    // Parse raw gyroscope data
    int16_t gyro_raw_x = (buffer[8] << 8) | buffer[9];
    int16_t gyro_raw_y = (buffer[10] << 8) | buffer[11];
    int16_t gyro_raw_z = (buffer[12] << 8) | buffer[13];
    
    // Convert to physical units
    // Accelerometer: g units then to m/s^2
    data->accel.x = ((float)accel_raw_x / hdev->accel_scale - hdev->accel_offset.x) * 9.81f;
    data->accel.y = ((float)accel_raw_y / hdev->accel_scale - hdev->accel_offset.y) * 9.81f;
    data->accel.z = ((float)accel_raw_z / hdev->accel_scale - hdev->accel_offset.z) * 9.81f;
    
    // Gyroscope: degrees/s then to rad/s
    data->gyro.x = ((float)gyro_raw_x / hdev->gyro_scale - hdev->gyro_offset.x) * (M_PI / 180.0f);
    data->gyro.y = ((float)gyro_raw_y / hdev->gyro_scale - hdev->gyro_offset.y) * (M_PI / 180.0f);
    data->gyro.z = ((float)gyro_raw_z / hdev->gyro_scale - hdev->gyro_offset.z) * (M_PI / 180.0f);
    
    // Temperature: convert using formula from datasheet
    data->temperature = (float)temp_raw / 340.0f + 36.53f;
    
    data->valid = true;
    return HAL_OK;
}

/* Calibrate gyroscope by averaging samples at rest */
HAL_StatusTypeDef MPU6050_Calibrate(MPU6050_Handle_t *hdev, uint16_t samples)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint8_t buffer[6];
    
    for (uint16_t i = 0; i < samples; i++) {
        HAL_StatusTypeDef status = MPU6050_ReadRegs(hdev, MPU6050_REG_GYRO_XOUT_H, buffer, 6);
        if (status != HAL_OK) return status;
        
        int16_t gx = (buffer[0] << 8) | buffer[1];
        int16_t gy = (buffer[2] << 8) | buffer[3];
        int16_t gz = (buffer[4] << 8) | buffer[5];
        
        sum_gx += (float)gx / hdev->gyro_scale;
        sum_gy += (float)gy / hdev->gyro_scale;
        sum_gz += (float)gz / hdev->gyro_scale;
        
        HAL_Delay(5);  // 200Hz sample rate
    }
    
    hdev->gyro_offset.x = sum_gx / (float)samples;
    hdev->gyro_offset.y = sum_gy / (float)samples;
    hdev->gyro_offset.z = sum_gz / (float)samples;
    
    return HAL_OK;
}
