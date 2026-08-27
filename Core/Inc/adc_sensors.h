/**
 * @file adc_sensors.h
 * @brief ADC-based current sensing (CJMCU-758 / ACS758LCB-100B)
 */
#ifndef ADC_SENSORS_H
#define ADC_SENSORS_H

#include "stm32l4xx_hal.h"
#include "flight_types.h"

/* CJMCU-758 OUT2 is connected to A5 / PC0 / ADC1_IN1. */
#define ADC_CHANNEL_CURRENT     ADC_CHANNEL_1

/* ACS758LCB-100B powered from 3.3 V, direct OUT2, no divider. */
#define CURRENT_SENSOR_SUPPLY_V 3.3f
#define CURRENT_SENSITIVITY     0.0132f // 20 mV/A * 3.3 V / 5.0 V
#define CURRENT_OFFSET_V        1.65f
#define CURRENT_POLARITY        (-1.0f) /* OUT2 falls as aircraft current rises. */

/* ADC reference */
#define ADC_VREF                3.3f
#define ADC_MAX_VALUE           4095    // 12-bit ADC

/* Power sensor handle */
typedef struct {
    ADC_HandleTypeDef *hadc;
    
    // Calibration
    float current_scale;
    float current_offset;
    
    // Filtered values
    float current_filtered;
    float filter_alpha;     // Low-pass filter coefficient (0-1)
    float ampere_hours;
    uint32_t last_sample_ms;
} PowerSensor_Handle_t;

/* Function prototypes */
void PowerSensor_Init(PowerSensor_Handle_t *hpow, ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef PowerSensor_Read(PowerSensor_Handle_t *hpow, Power_Data_t *data);
void PowerSensor_CalibrateZeroCurrent(PowerSensor_Handle_t *hpow);

/* Low-level ADC functions */
uint32_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel);
float ADC_ToVoltage(uint32_t adc_value);
HAL_StatusTypeDef MCU_Temperature_Read(ADC_HandleTypeDef *hadc,
                                       float *temperature_c);

#endif /* ADC_SENSORS_H */
