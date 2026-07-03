/**
 * @file adc_sensors.c
 * @brief ADC-based ACS758 current sensing implementation
 */
#include "adc_sensors.h"

void PowerSensor_Init(PowerSensor_Handle_t *hpow, ADC_HandleTypeDef *hadc)
{
    hpow->hadc = hadc;
    hpow->current_scale = ADC_VREF / ((float)ADC_MAX_VALUE * CURRENT_SENSITIVITY);
    hpow->current_offset = CURRENT_OFFSET_V;
    hpow->current_filtered = 0.0f;
    hpow->filter_alpha = 0.1f;
    HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
}

uint32_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK ||
        HAL_ADC_Start(hadc) != HAL_OK) {
        return 0;
    }
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
        uint32_t value = HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
        return value;
    }
    HAL_ADC_Stop(hadc);
    return 0;
}

float ADC_ToVoltage(uint32_t adc_value)
{
    return (float)adc_value * ADC_VREF / (float)ADC_MAX_VALUE;
}

HAL_StatusTypeDef PowerSensor_Read(PowerSensor_Handle_t *hpow, Power_Data_t *data)
{
    /* ACS758 OUT1 on A5 / PC0 / ADC1_IN1. */
    uint32_t current_raw = ADC_ReadChannel(hpow->hadc, ADC_CHANNEL_CURRENT);
    float current_adc = ADC_ToVoltage(current_raw);
    float current = (current_adc - hpow->current_offset) / CURRENT_SENSITIVITY;

    hpow->current_filtered = hpow->filter_alpha * current +
                             (1.0f - hpow->filter_alpha) * hpow->current_filtered;

    data->voltage = 0.0f; /* Battery-voltage sensing intentionally not fitted. */
    data->current = hpow->current_filtered;
    data->power = 0.0f;   /* Cannot calculate power without battery voltage. */
    data->valid = true;
    return HAL_OK;
}

void PowerSensor_CalibrateZeroCurrent(PowerSensor_Handle_t *hpow)
{
    uint64_t sum = 0;
    const uint16_t samples = 128;
    for (uint16_t i = 0; i < samples; ++i) {
        sum += ADC_ReadChannel(hpow->hadc, ADC_CHANNEL_CURRENT);
        HAL_Delay(2);
    }
    hpow->current_offset = ADC_ToVoltage((uint32_t)(sum / samples));
    hpow->current_filtered = 0.0f;
}
