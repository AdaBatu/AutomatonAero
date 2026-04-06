/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flight_types.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "gps_nmea.h"
#include "sx1278.h"
#include "kalman.h"
#include "pid.h"
#include "servo_esc.h"
#include "adc_sensors.h"
#include "telemetry.h"
#include "rc_input.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* LoRa SX1278 GPIO Pins */
#define LORA_NSS_PIN        GPIO_PIN_7
#define LORA_NSS_PORT       GPIOC
#define LORA_RST_PIN        GPIO_PIN_10
#define LORA_RST_PORT       GPIOB
#define LORA_DIO0_PIN       GPIO_PIN_9
#define LORA_DIO0_PORT      GPIOC

/* GPS PPS (Pulse Per Second) Pin */
#define GPS_PPS_PIN         GPIO_PIN_4
#define GPS_PPS_PORT        GPIOA

/* ESC Programming Pin (Skywalker 100A V2-UBEC) */
#define ESC_PROG_PIN        GPIO_PIN_10
#define ESC_PROG_PORT       GPIOC

/* Control loop frequency */
#define CONTROL_LOOP_HZ     100  // 100 Hz main control loop

/* GPS UART baud rate (NEO-6M default) */
#define GPS_UART_BAUD       9600

/* USER CODE END Private defines */

/* RC Receiver Input Pins */
#define RC_THROTTLE_PIN     GPIO_PIN_1
#define RC_THROTTLE_PORT    GPIOB
#define RC_ROLL_PIN         GPIO_PIN_2
#define RC_ROLL_PORT        GPIOB
#define RC_PITCH_PIN        GPIO_PIN_15
#define RC_PITCH_PORT       GPIOB
#define RC_YAW_PIN          GPIO_PIN_11
#define RC_YAW_PORT         GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
