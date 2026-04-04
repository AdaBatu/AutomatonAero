/**
 * @file adc_sensors.h
 * @brief ADC-based voltage and current sensing (WCMCU-758)
 */
#ifndef ADC_SENSORS_H
#define ADC_SENSORS_H

#include "stm32l4xx_hal.h"
#include "flight_types.h"

/* ADC Channel assignments */
#define ADC_CHANNEL_VOLTAGE     ADC_CHANNEL_1   // PC0
#define ADC_CHANNEL_CURRENT     ADC_CHANNEL_2   // PC1

/* Calibration constants for WCMCU-758 */
// Adjust these based on your specific module and voltage divider
#define VOLTAGE_DIVIDER_RATIO   11.0f   // (R1 + R2) / R2 for voltage measurement
#define CURRENT_SENSITIVITY     0.066f  // V per A for ACS712-30A (adjust for your sensor)
#define CURRENT_OFFSET_V        2.5f    // Zero current output (Vcc/2)

/* ADC reference */
#define ADC_VREF                3.3f
#define ADC_MAX_VALUE           4095    // 12-bit ADC

/* Power sensor handle */
typedef struct {
    ADC_HandleTypeDef *hadc;
    
    // Calibration
    float voltage_scale;
    float voltage_offset;
    float current_scale;
    float current_offset;
    
    // Filtered values
    float voltage_filtered;
    float current_filtered;
    float filter_alpha;     // Low-pass filter coefficient (0-1)
} PowerSensor_Handle_t;

/* Function prototypes */
void PowerSensor_Init(PowerSensor_Handle_t *hpow, ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef PowerSensor_Read(PowerSensor_Handle_t *hpow, Power_Data_t *data);
void PowerSensor_Calibrate(PowerSensor_Handle_t *hpow, float measured_voltage, float measured_current);

/* Low-level ADC functions */
uint32_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel);
float ADC_ToVoltage(uint32_t adc_value);

#endif /* ADC_SENSORS_H */
