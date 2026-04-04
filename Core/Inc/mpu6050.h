/**
 * @file mpu6050.h
 * @brief MPU-6050 IMU driver for STM32
 */
#ifndef MPU6050_H
#define MPU6050_H

#include "stm32l4xx_hal.h"
#include "flight_types.h"

/* MPU-6050 I2C Address */
#define MPU6050_ADDR            (0x68 << 1)  // AD0 = LOW
#define MPU6050_ADDR_ALT        (0x69 << 1)  // AD0 = HIGH

/* Register addresses */
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_FIFO_EN         0x23
#define MPU6050_REG_INT_PIN_CFG     0x37
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_INT_STATUS      0x3A
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_USER_CTRL       0x6A
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_WHO_AM_I        0x75

/* Configuration values */
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x08
#define MPU6050_GYRO_FS_1000        0x10
#define MPU6050_GYRO_FS_2000        0x18

#define MPU6050_ACCEL_FS_2G         0x00
#define MPU6050_ACCEL_FS_4G         0x08
#define MPU6050_ACCEL_FS_8G         0x10
#define MPU6050_ACCEL_FS_16G        0x18

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

/* Scale factors */
#define MPU6050_ACCEL_SCALE_2G      16384.0f
#define MPU6050_ACCEL_SCALE_4G      8192.0f
#define MPU6050_ACCEL_SCALE_8G      4096.0f
#define MPU6050_ACCEL_SCALE_16G     2048.0f

#define MPU6050_GYRO_SCALE_250      131.0f
#define MPU6050_GYRO_SCALE_500      65.5f
#define MPU6050_GYRO_SCALE_1000     32.8f
#define MPU6050_GYRO_SCALE_2000     16.4f

/* Handle structure */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    float accel_scale;
    float gyro_scale;
    Vector3f_t gyro_offset;
    Vector3f_t accel_offset;
} MPU6050_Handle_t;

/* Function prototypes */
HAL_StatusTypeDef MPU6050_Init(MPU6050_Handle_t *hdev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_Handle_t *hdev, IMU_Data_t *data);
HAL_StatusTypeDef MPU6050_Calibrate(MPU6050_Handle_t *hdev, uint16_t samples);
uint8_t MPU6050_WhoAmI(MPU6050_Handle_t *hdev);

#endif /* MPU6050_H */
