/**
 * @file ms5611.h
 * @brief MS5611 Barometer driver for STM32 (GY-63 module)
 */
#ifndef MS5611_H
#define MS5611_H

#include "stm32l4xx_hal.h"
#include "flight_types.h"

/* MS5611 I2C Addresses */
#define MS5611_ADDR_CSB_LOW     (0x77 << 1)  // CSB = LOW
#define MS5611_ADDR_CSB_HIGH    (0x76 << 1)  // CSB = HIGH

/* Commands */
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_PROM_READ    0xA0  // Base address, add 0-7
#define MS5611_CMD_CONV_D1_256  0x40  // Convert pressure OSR=256
#define MS5611_CMD_CONV_D1_512  0x42
#define MS5611_CMD_CONV_D1_1024 0x44
#define MS5611_CMD_CONV_D1_2048 0x46
#define MS5611_CMD_CONV_D1_4096 0x48
#define MS5611_CMD_CONV_D2_256  0x50  // Convert temperature OSR=256
#define MS5611_CMD_CONV_D2_512  0x52
#define MS5611_CMD_CONV_D2_1024 0x54
#define MS5611_CMD_CONV_D2_2048 0x56
#define MS5611_CMD_CONV_D2_4096 0x58
#define MS5611_CMD_ADC_READ     0x00

/* Oversampling settings */
typedef enum {
    MS5611_OSR_256  = 0,
    MS5611_OSR_512  = 1,
    MS5611_OSR_1024 = 2,
    MS5611_OSR_2048 = 3,
    MS5611_OSR_4096 = 4
} MS5611_OSR_t;

/* Handle structure */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    uint16_t prom[8];       // Calibration coefficients
    uint32_t D1;            // Raw pressure
    uint32_t D2;            // Raw temperature
    float reference_pressure;  // For altitude calculation
    MS5611_OSR_t osr;
} MS5611_Handle_t;

/* Function prototypes */
HAL_StatusTypeDef MS5611_Init(MS5611_Handle_t *hdev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MS5611_StartConvPressure(MS5611_Handle_t *hdev);
HAL_StatusTypeDef MS5611_StartConvTemperature(MS5611_Handle_t *hdev);
HAL_StatusTypeDef MS5611_ReadADC(MS5611_Handle_t *hdev, uint32_t *value);
HAL_StatusTypeDef MS5611_Calculate(MS5611_Handle_t *hdev, Baro_Data_t *data);
HAL_StatusTypeDef MS5611_ReadBlocking(MS5611_Handle_t *hdev, Baro_Data_t *data);
void MS5611_SetReferencePressure(MS5611_Handle_t *hdev, float pressure);

#endif /* MS5611_H */
