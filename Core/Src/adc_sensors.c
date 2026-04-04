/**
 * @file adc_sensors.c
 * @brief ADC-based voltage and current sensing implementation
 */
#include "adc_sensors.h"

/* Initialize power sensor */
void PowerSensor_Init(PowerSensor_Handle_t *hpow, ADC_HandleTypeDef *hadc)
{
    hpow->hadc = hadc;
    
    // Default calibration for WCMCU-758 / INA-758
    // Voltage divider: typically 11:1 for measuring up to 25V
    hpow->voltage_scale = VOLTAGE_DIVIDER_RATIO * ADC_VREF / (float)ADC_MAX_VALUE;
    hpow->voltage_offset = 0.0f;
    
    // Current sensor: typically ACS712 or similar
    // Sensitivity depends on model (5A, 20A, 30A versions)
    hpow->current_scale = ADC_VREF / ((float)ADC_MAX_VALUE * CURRENT_SENSITIVITY);
    hpow->current_offset = CURRENT_OFFSET_V / CURRENT_SENSITIVITY;  // Zero point in Amps
    
    // Initialize filtered values
    hpow->voltage_filtered = 0.0f;
    hpow->current_filtered = 0.0f;
    hpow->filter_alpha = 0.1f;  // Low-pass filter coefficient
    
    // Calibrate ADC
    HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
}

/* Read single ADC channel */
uint32_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // Configure channel
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    
    HAL_ADC_ConfigChannel(hadc, &sConfig);
    
    // Start conversion
    HAL_ADC_Start(hadc);
    
    // Wait for conversion
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
        uint32_t value = HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
        return value;
    }
    
    HAL_ADC_Stop(hadc);
    return 0;
}

/* Convert ADC value to voltage */
float ADC_ToVoltage(uint32_t adc_value)
{
    return (float)adc_value * ADC_VREF / (float)ADC_MAX_VALUE;
}

/* Read power sensor data */
HAL_StatusTypeDef PowerSensor_Read(PowerSensor_Handle_t *hpow, Power_Data_t *data)
{
    // Read voltage channel (PC0 = ADC1_IN1)
    uint32_t voltage_raw = ADC_ReadChannel(hpow->hadc, ADC_CHANNEL_VOLTAGE);
    float voltage_adc = ADC_ToVoltage(voltage_raw);
    float voltage = (voltage_adc * VOLTAGE_DIVIDER_RATIO) - hpow->voltage_offset;
    
    // Read current channel (PC1 = ADC1_IN2)
    uint32_t current_raw = ADC_ReadChannel(hpow->hadc, ADC_CHANNEL_CURRENT);
    float current_adc = ADC_ToVoltage(current_raw);
    
    // Current sensor output: Vout = Vref/2 + (I * sensitivity)
    // For ACS712-30A: sensitivity = 66mV/A, zero current = Vcc/2 = 2.5V
    float current = (current_adc - CURRENT_OFFSET_V) / CURRENT_SENSITIVITY;
    
    // Apply low-pass filter
    hpow->voltage_filtered = hpow->filter_alpha * voltage + 
                             (1.0f - hpow->filter_alpha) * hpow->voltage_filtered;
    hpow->current_filtered = hpow->filter_alpha * current + 
                             (1.0f - hpow->filter_alpha) * hpow->current_filtered;
    
    // Store results
    data->voltage = hpow->voltage_filtered;
    data->current = hpow->current_filtered;
    data->power = data->voltage * data->current;
    data->valid = true;
    
    return HAL_OK;
}

/* Calibrate power sensor with known values */
void PowerSensor_Calibrate(PowerSensor_Handle_t *hpow, float measured_voltage, float measured_current)
{
    // Read current raw values
    uint32_t voltage_raw = ADC_ReadChannel(hpow->hadc, ADC_CHANNEL_VOLTAGE);
    uint32_t current_raw = ADC_ReadChannel(hpow->hadc, ADC_CHANNEL_CURRENT);
    
    float voltage_adc = ADC_ToVoltage(voltage_raw);
    float current_adc = ADC_ToVoltage(current_raw);
    
    // Calculate offsets
    // For voltage: measured = (adc * scale) - offset
    // offset = (adc * scale) - measured
    if (measured_voltage > 0.0f) {
        hpow->voltage_offset = (voltage_adc * VOLTAGE_DIVIDER_RATIO) - measured_voltage;
    }
    
    // For current: measured = (adc - Vref/2) / sensitivity
    // This is more complex - typically just verify zero-current reading
    if (measured_current == 0.0f) {
        // At zero current, ADC should read ~2.5V (Vcc/2)
        // Store offset for correction
        hpow->current_offset = (current_adc - CURRENT_OFFSET_V) / CURRENT_SENSITIVITY;
    }
    
    // Reset filtered values
    hpow->voltage_filtered = measured_voltage;
    hpow->current_filtered = measured_current;
}
