/**
 * @file adc_sensors.c
 * @brief ADC-based ACS758 current sensing implementation
 */
#include "adc_sensors.h"
#include "flight_config_generated.h"
#include <stdio.h>

void PowerSensor_Init(PowerSensor_Handle_t *hpow, ADC_HandleTypeDef *hadc)
{
    hpow->hadc = hadc;
    hpow->current_scale = ADC_VREF / ((float)ADC_MAX_VALUE * CURRENT_SENSITIVITY);
    hpow->current_offset = CURRENT_OFFSET_V;
    hpow->current_filtered = 0.0f;
    hpow->filter_alpha = 0.1f;
    hpow->ampere_hours = 0.0f;
    hpow->last_sample_ms = HAL_GetTick();
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

HAL_StatusTypeDef MCU_Temperature_Read(ADC_HandleTypeDef *hadc,
                                       float *temperature_c)
{
    if (hadc == NULL || temperature_c == NULL) return HAL_ERROR;

    ADC_ChannelConfTypeDef config = {0};
    config.Channel = ADC_CHANNEL_TEMPSENSOR;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset = 0;
    if (HAL_ADC_ConfigChannel(hadc, &config) != HAL_OK) return HAL_ERROR;

    uint32_t sum = 0U;
    for (uint8_t i = 0U; i < 8U; ++i) {
        if (HAL_ADC_Start(hadc) != HAL_OK) return HAL_ERROR;
        if (HAL_ADC_PollForConversion(hadc, 10U) != HAL_OK) {
            HAL_ADC_Stop(hadc);
            return HAL_TIMEOUT;
        }
        sum += HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
    }

    uint32_t average = sum / 8U;
    *temperature_c = (float)__HAL_ADC_CALC_TEMPERATURE(
        3300U, average, ADC_RESOLUTION_12B);
    return HAL_OK;
}

HAL_StatusTypeDef PowerSensor_Read(PowerSensor_Handle_t *hpow, Power_Data_t *data)
{
    /* CJMCU-758 OUT2 on A5 / PC0 / ADC1_IN1. */
    uint32_t current_raw = ADC_ReadChannel(hpow->hadc, ADC_CHANNEL_CURRENT);
    float current_adc = ADC_ToVoltage(current_raw);
    float current = CURRENT_POLARITY *
                    (current_adc - hpow->current_offset) / CURRENT_SENSITIVITY;

    hpow->current_filtered = hpow->filter_alpha * current +
                             (1.0f - hpow->filter_alpha) * hpow->current_filtered;

    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = now_ms - hpow->last_sample_ms;
    hpow->last_sample_ms = now_ms;

    /* Accumulate consumed capacity/energy. Reverse current is not consumption. */
    float consumed_current = hpow->current_filtered > 0.0f ?
                             hpow->current_filtered : 0.0f;
    float elapsed_hours = (float)elapsed_ms / 3600000.0f;
    float power_w = CONFIG_BATTERY_VOLTAGE_V * hpow->current_filtered;
    hpow->ampere_hours += consumed_current * elapsed_hours;

    data->voltage = CONFIG_BATTERY_VOLTAGE_V;
    data->current = hpow->current_filtered;
    data->current_sensor_voltage = current_adc;
    data->power = power_w;
    data->ampere_hours = hpow->ampere_hours;
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
    printf("Current sensor zero: %.4f V on PC0 (ADC1_IN1)\r\n",
           hpow->current_offset);
    hpow->current_filtered = 0.0f;
    hpow->last_sample_ms = HAL_GetTick();
}
