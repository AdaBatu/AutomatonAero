/**
 * @file ms5611.c
 * @brief MS5611 Barometer driver implementation (GY-63)
 */
#include "ms5611.h"
#include <math.h>

#define I2C_TIMEOUT     100

/* Conversion delay in microseconds based on OSR */
static const uint16_t conv_delay_us[] = { 600, 1200, 2300, 4600, 9100 };

/* Send command to MS5611 */
static HAL_StatusTypeDef MS5611_Command(MS5611_Handle_t *hdev, uint8_t cmd)
{
    return HAL_I2C_Master_Transmit(hdev->hi2c, hdev->address, &cmd, 1, I2C_TIMEOUT);
}

/* Read PROM calibration data */
static HAL_StatusTypeDef MS5611_ReadPROM(MS5611_Handle_t *hdev)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;
    
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t cmd = MS5611_CMD_PROM_READ + (i * 2);
        status = HAL_I2C_Master_Transmit(hdev->hi2c, hdev->address, &cmd, 1, I2C_TIMEOUT);
        if (status != HAL_OK) return status;
        
        status = HAL_I2C_Master_Receive(hdev->hi2c, hdev->address, data, 2, I2C_TIMEOUT);
        if (status != HAL_OK) return status;
        
        hdev->prom[i] = (data[0] << 8) | data[1];
    }
    
    return HAL_OK;
}

/* Initialize MS5611 */
HAL_StatusTypeDef MS5611_Init(MS5611_Handle_t *hdev, I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    
    hdev->hi2c = hi2c;
    hdev->address = MS5611_ADDR_CSB_LOW;  // Default address
    hdev->osr = MS5611_OSR_4096;          // Highest resolution
    hdev->reference_pressure = 101325.0f; // Standard sea level pressure
    
    // Reset device
    status = MS5611_Command(hdev, MS5611_CMD_RESET);
    if (status != HAL_OK) {
        // Try alternate address
        hdev->address = MS5611_ADDR_CSB_HIGH;
        status = MS5611_Command(hdev, MS5611_CMD_RESET);
        if (status != HAL_OK) return status;
    }
    
    HAL_Delay(10);  // Wait for reset
    
    // Read calibration data
    status = MS5611_ReadPROM(hdev);
    if (status != HAL_OK) return status;
    
    hdev->D1 = 0;
    hdev->D2 = 0;
    
    return HAL_OK;
}

/* Start pressure conversion */
HAL_StatusTypeDef MS5611_StartConvPressure(MS5611_Handle_t *hdev)
{
    uint8_t cmd = MS5611_CMD_CONV_D1_256 + (hdev->osr * 2);
    return MS5611_Command(hdev, cmd);
}

/* Start temperature conversion */
HAL_StatusTypeDef MS5611_StartConvTemperature(MS5611_Handle_t *hdev)
{
    uint8_t cmd = MS5611_CMD_CONV_D2_256 + (hdev->osr * 2);
    return MS5611_Command(hdev, cmd);
}

/* Read ADC result */
HAL_StatusTypeDef MS5611_ReadADC(MS5611_Handle_t *hdev, uint32_t *value)
{
    uint8_t cmd = MS5611_CMD_ADC_READ;
    uint8_t data[3];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Transmit(hdev->hi2c, hdev->address, &cmd, 1, I2C_TIMEOUT);
    if (status != HAL_OK) return status;
    
    status = HAL_I2C_Master_Receive(hdev->hi2c, hdev->address, data, 3, I2C_TIMEOUT);
    if (status != HAL_OK) return status;
    
    *value = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    return HAL_OK;
}

/* Calculate temperature and pressure from raw values */
HAL_StatusTypeDef MS5611_Calculate(MS5611_Handle_t *hdev, Baro_Data_t *data)
{
    // Using coefficients from PROM
    uint16_t C1 = hdev->prom[1];  // Pressure sensitivity
    uint16_t C2 = hdev->prom[2];  // Pressure offset
    uint16_t C3 = hdev->prom[3];  // Temp coefficient of pressure sensitivity
    uint16_t C4 = hdev->prom[4];  // Temp coefficient of pressure offset
    uint16_t C5 = hdev->prom[5];  // Reference temperature
    uint16_t C6 = hdev->prom[6];  // Temp coefficient of temperature
    
    // Calculate temperature
    int32_t dT = (int32_t)hdev->D2 - ((int32_t)C5 << 8);
    int32_t TEMP = 2000 + (((int64_t)dT * C6) >> 23);
    
    // Calculate temperature compensated pressure
    int64_t OFF = ((int64_t)C2 << 16) + (((int64_t)C4 * dT) >> 7);
    int64_t SENS = ((int64_t)C1 << 15) + (((int64_t)C3 * dT) >> 8);
    
    // Second order temperature compensation
    int32_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;
    
    if (TEMP < 2000) {
        T2 = ((int64_t)dT * dT) >> 31;
        OFF2 = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 1;
        SENS2 = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 2;
        
        if (TEMP < -1500) {
            OFF2 += 7 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += (11 * (TEMP + 1500) * (TEMP + 1500)) >> 1;
        }
    }
    
    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
    
    // Calculate pressure (Pa)
    // Formula: P = D1×SENS/2^21 − OFF/2^15
    int32_t P = (((int64_t)hdev->D1 * SENS) >> 21) - (OFF >> 15);
    
    // Store results
    data->temperature = (float)TEMP / 100.0f;  // Celsius
    data->pressure = (float)P;                  // Pa
    
    // Calculate altitude using barometric formula
    // h = 44330 * (1 - (P/P0)^0.1903)
    data->altitude = 44330.0f * (1.0f - powf(data->pressure / hdev->reference_pressure, 0.1903f));
    
    data->valid = true;
    return HAL_OK;
}

/* Read pressure and temperature (blocking) */
HAL_StatusTypeDef MS5611_ReadBlocking(MS5611_Handle_t *hdev, Baro_Data_t *data)
{
    HAL_StatusTypeDef status;
    
    // Convert D1 (pressure)
    status = MS5611_StartConvPressure(hdev);
    if (status != HAL_OK) return status;
    
    HAL_Delay((conv_delay_us[hdev->osr] / 1000) + 1);
    
    status = MS5611_ReadADC(hdev, &hdev->D1);
    if (status != HAL_OK) return status;
    
    // Convert D2 (temperature)
    status = MS5611_StartConvTemperature(hdev);
    if (status != HAL_OK) return status;
    
    HAL_Delay((conv_delay_us[hdev->osr] / 1000) + 1);
    
    status = MS5611_ReadADC(hdev, &hdev->D2);
    if (status != HAL_OK) return status;
    
    // Calculate final values
    return MS5611_Calculate(hdev, data);
}

/* Set reference pressure for altitude calculation */
void MS5611_SetReferencePressure(MS5611_Handle_t *hdev, float pressure)
{
    hdev->reference_pressure = pressure;
}
