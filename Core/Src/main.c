/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

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
#include "serial_telemetry.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* Definitions for flightTask */
osThreadId_t flightTaskHandle;
const osThreadAttr_t flightTask_attributes = {
  .name = "flightTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for telemetryTask */
osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
  .name = "telemetryTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorsTask */
osThreadId_t sensorsTaskHandle;
const osThreadAttr_t sensorsTask_attributes = {
  .name = "sensorsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */

/* Sensor handles */
MPU6050_Handle_t hmpu;
MS5611_Handle_t hbaro;
GPS_Handle_t hgps;
SX1278_Handle_t hlora;
PowerSensor_Handle_t hpower;

/* Control system */
KalmanFilter_t kalman;
FlightPID_t flight_pid;

/* Actuator handles */
Servo_Handle_t servo_roll;   // TIM1_CH1 - PA8
Servo_Handle_t servo_pitch;  // TIM1_CH2 - PA9
Servo_Handle_t servo_yaw;    // TIM1_CH3 - PA10
ESC_Handle_t esc;            // TIM2_CH2 - PB3
// NOTE: Throttle now on TIM2_CH2 (PB3), not TIM2_CH1 (PA15)

/* Telemetry */
Telemetry_Handle_t htelemetry;

/* Serial Debug Telemetry */
SerialTelemetry_Handle_t hserial_telem;

/* RC Input */
RC_Handle_t hrc;

/* Flight state */
FlightState_t flight_state;

/* Control parameters */
float throttle_command = 0.0f;  // 0.0 to 1.0
volatile uint8_t system_armed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
void StartflightTask(void *argument);
void StarttelemetryTask(void *argument);
void StartsensorsTask(void *argument);

/* USER CODE BEGIN PFP */
static void Flight_InitSensors(void);
static void Flight_InitActuators(void);
static void Flight_InitRadio(void);
static void Flight_InitRCInput(void);
static void Flight_ControlLoop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  /* Initialize flight state */
  memset(&flight_state, 0, sizeof(FlightState_t));
  
  /* Initialize Kalman filter and PID controllers */
  Kalman_Init(&kalman);
  Kalman_SetTuning(&kalman, 0.001f, 0.003f, 0.03f);  // Q_angle, Q_bias, R_measure
  
  FlightPID_Init(&flight_pid);
  
  /* Initialize sensors */
  Flight_InitSensors();
  
  /* Initialize RC receiver input */
  Flight_InitRCInput();
  
  /* Initialize actuators */
  Flight_InitActuators();
  
  /* Initialize LoRa radio */
  Flight_InitRadio();
  
  /* Initialize serial debug telemetry (5Hz = print every 200ms) */
  SerialTelemetry_Init(&hserial_telem, 5);
  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of flightTask */
  flightTaskHandle = osThreadNew(StartflightTask, NULL, &flightTask_attributes);

  /* creation of telemetryTask */
  telemetryTaskHandle = osThreadNew(StarttelemetryTask, NULL, &telemetryTask_attributes);

  /* creation of sensorsTask */
  sensorsTaskHandle = osThreadNew(StartsensorsTask, NULL, &sensorsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1600-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1600-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB11 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Initialize all sensors
 */
static void Flight_InitSensors(void)
{
    /* Configure PA4 as input for GPS PPS */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPS_PPS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  // Interrupt on rising edge
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPS_PPS_PORT, &GPIO_InitStruct);
    
    /* Enable EXTI interrupt for PPS */
    HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    
    /* Initialize MPU-6050 IMU on I2C2 */
    if (MPU6050_Init(&hmpu, &hi2c2) == HAL_OK) {
        /* Calibrate gyroscope (keep device still!) */
        MPU6050_Calibrate(&hmpu, 200);
    }
    
    /* Initialize MS5611 Barometer on I2C2 */
    MS5611_Init(&hbaro, &hi2c2);
    
    /* Initialize GPS on UART4 with PPS on PA4 */
    GPS_InitWithPPS(&hgps, &huart4, GPS_PPS_PORT, GPS_PPS_PIN);
    
    /* Initialize power sensor on ADC1 */
    PowerSensor_Init(&hpower, &hadc1);
}

/**
 * @brief Initialize servos and ESC
 */
static void Flight_InitActuators(void)
{
    /* Initialize servos on TIM1 */
    Servo_Init(&servo_roll, &htim1, TIM_CHANNEL_1);   // PA8
    Servo_Init(&servo_pitch, &htim1, TIM_CHANNEL_2);  // PA9
    Servo_Init(&servo_yaw, &htim1, TIM_CHANNEL_3);    // PA10
    
    /* Configure PC10 as output for ESC programming */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = ESC_PROG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ESC_PROG_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(ESC_PROG_PORT, ESC_PROG_PIN, GPIO_PIN_RESET);
    
    /* Initialize ESC on TIM2 with programming pin on PC10 */
    // Throttle output now on TIM2_CH2 (PB3)
    ESC_InitWithProg(&esc, &htim2, TIM_CHANNEL_2, ESC_PROG_PORT, ESC_PROG_PIN);
    
    /* Center all servos */
    Servo_SetAngle(&servo_roll, 0.0f);
    Servo_SetAngle(&servo_pitch, 0.0f);
    Servo_SetAngle(&servo_yaw, 0.0f);
}

/**
 * @brief Initialize LoRa radio
 */
static void Flight_InitRadio(void)
{
    /* Configure GPIO for LoRa control pins */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* NSS pin (PC7) - Output */
    GPIO_InitStruct.Pin = LORA_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LORA_NSS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
    
    /* RST pin (PB10) - Output */
    GPIO_InitStruct.Pin = LORA_RST_PIN;
    HAL_GPIO_Init(LORA_RST_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    
    /* Initialize SX1278 */
    if (SX1278_Init(&hlora, &hspi1, 
                    LORA_NSS_PORT, LORA_NSS_PIN,
                    LORA_RST_PORT, LORA_RST_PIN,
                    LORA_DIO0_PORT, LORA_DIO0_PIN) == HAL_OK) {
        
        /* Configure LoRa parameters */
        SX1278_Config_t lora_config = {
            .frequency = 433000000,           // 433 MHz
            .bandwidth = SX1278_BW_125_KHZ,   // 125 kHz bandwidth
            .spreading_factor = SX1278_SF_7,  // SF7 for faster data rate
            .coding_rate = SX1278_CR_4_5,     // 4/5 coding rate
            .tx_power = 17,                   // 17 dBm
            .preamble_length = 8,
            .sync_word = 0x12                 // Private network
        };
        
        SX1278_Configure(&hlora, &lora_config);
    }
    
    /* Initialize telemetry */
    Telemetry_Init(&htelemetry, &hlora);
    Telemetry_SetRate(&htelemetry, 10);  // 10 Hz telemetry
}

/**
 * @brief Initialize RC receiver input
 */
static void Flight_InitRCInput(void)
{
    RC_Init(&hrc);
}

/**
 * @brief Main flight control loop - called from RTOS task
 */
static void Flight_ControlLoop(void)
{
    static uint32_t last_loop_time = 0;
    static uint32_t last_baro_time = 0;
    static uint32_t last_power_time = 0;
    
    uint32_t now = HAL_GetTick();
    float dt = (now - last_loop_time) / 1000.0f;
    
    if (dt < 0.001f) dt = 0.01f;  // Prevent division issues
    last_loop_time = now;
    
    /* ========== READ RC INPUT ========== */
    RC_Update(&hrc);
    RC_Input_t rc_input;
    RC_GetInput(&hrc, &rc_input);
    
    /* ========== READ IMU (High Priority - 100Hz+) ========== */
    if (MPU6050_ReadAll(&hmpu, &flight_state.imu) == HAL_OK) {
        flight_state.last_imu_update = now;
        
        /* Update Kalman filter */
        Kalman_Update(&kalman, 
                      &flight_state.imu.accel, 
                      &flight_state.imu.gyro, 
                      dt);
        
        /* Get filtered orientation */
        Kalman_GetOrientation(&kalman, &flight_state.orientation);
    }
    
    /* ========== READ BAROMETER (10Hz) ========== */
    if (now - last_baro_time >= 100) {
        last_baro_time = now;
        MS5611_ReadBlocking(&hbaro, &flight_state.baro);
    }
    
    /* ========== READ POWER SENSOR (5Hz) ========== */
    if (now - last_power_time >= 200) {
        last_power_time = now;
        PowerSensor_Read(&hpower, &flight_state.power);
    }
    
    /* ========== UPDATE PID SETPOINTS FROM RC ========== */
    /* Roll and Pitch: RC stick -> PID target angle (±30 degrees max) */
    float target_roll = RC_StickToAngle(rc_input.roll, 30.0f * 3.14159265f / 180.0f);
    float target_pitch = RC_StickToAngle(rc_input.pitch, 30.0f * 3.14159265f / 180.0f);
    
    /* Set PID targets (yaw setpoint not used - direct passthrough) */
    FlightPID_SetSetpoint(&flight_pid, target_roll, target_pitch, 0.0f);
    
    /* ========== PID CONTROL (100Hz) ========== */
    FlightPID_Update(&flight_pid, &flight_state.orientation, dt);
    
    /* Get PID outputs for roll and pitch */
    float roll_out, pitch_out, yaw_pid_out;
    FlightPID_GetOutputs(&flight_pid, &roll_out, &pitch_out, &yaw_pid_out);
    
    /* Yaw and Throttle: Direct passthrough from RC (1:1) */
    float yaw_out = rc_input.yaw;
    throttle_command = rc_input.throttle;
    
    /* ========== ACTUATOR OUTPUT ========== */
    if (system_armed) {
        /* Apply PID outputs to roll/pitch servos (normalized -1 to +1) */
        Servo_SetNormalized(&servo_roll, roll_out);
        Servo_SetNormalized(&servo_pitch, pitch_out);
        
        /* Apply direct RC yaw (1:1 passthrough) */
        Servo_SetNormalized(&servo_yaw, yaw_out);
        
        /* Set ESC throttle (direct from RC, 1:1 passthrough) */
        ESC_SetThrottle(&esc, throttle_command);
    } else {
        /* Disarmed - center servos, zero throttle */
        Servo_SetNormalized(&servo_roll, 0.0f);
        Servo_SetNormalized(&servo_pitch, 0.0f);
        Servo_SetNormalized(&servo_yaw, 0.0f);
    }
    
    /* ========== TELEMETRY (10Hz) ========== */
    if (Telemetry_ReadyToSend(&htelemetry)) {
        /* Convert servo outputs to 0-255 range for telemetry */
        uint8_t telem_roll = (uint8_t)((roll_out + 1.0f) * 127.5f);
        uint8_t telem_pitch = (uint8_t)((pitch_out + 1.0f) * 127.5f);
        uint8_t telem_yaw = (uint8_t)((yaw_out + 1.0f) * 127.5f);
        uint8_t telem_throttle = (uint8_t)(throttle_command * 255.0f);
        
        Telemetry_BuildPacket(&htelemetry, &flight_state,
                              telem_roll, telem_pitch, telem_yaw, telem_throttle);
        Telemetry_Send(&htelemetry);
    }
    
    /* Update telemetry state */
    Telemetry_Update(&htelemetry);
    
    /* ========== SERIAL DEBUG TELEMETRY ========== */
    #ifdef SERIAL_TELEMETRY_ENABLED
    if (SerialTelemetry_ReadyToPrint(&hserial_telem)) {
        SerialTelemetry_Print(&hserial_telem, &flight_state, 
                            roll_out, pitch_out, yaw_out, throttle_command);
    }
    #endif
    
    flight_state.loop_count++;
}

/* GPS UART Receive Callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        GPS_IRQHandler(&hgps);
        flight_state.last_gps_update = HAL_GetTick();
    }
}

/* GPS PPS EXTI Callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPS_PPS_PIN) {
        GPS_PPS_IRQHandler(&hgps);
    }
    
    /* RC receiver input pins (PB1, PB2, PB11, PB15) */
    if (GPIO_Pin == GPIO_PIN_1 || GPIO_Pin == GPIO_PIN_2 ||
        GPIO_Pin == GPIO_PIN_11 || GPIO_Pin == GPIO_PIN_15) {
        RC_EXTI_Handler(&hrc, GPIO_Pin);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartflightTask */
/**
  * @brief  Function implementing the flightTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartflightTask */
void StartflightTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  
  /* Give sensors time to stabilize */
  osDelay(1000);
  
  /* Arm ESC (keep at minimum throttle for 2 seconds) */
  ESC_Arm(&esc);
  osDelay(2000);
  
  /* Set initial setpoint (level flight) */
  FlightPID_SetSetpoint(&flight_pid, 0.0f, 0.0f, 0.0f);
  
  /* Main flight control loop */
  uint32_t last_tick = osKernelGetTickCount();
  const uint32_t loop_period_ms = 1000 / CONTROL_LOOP_HZ;  // 10ms for 100Hz
  
  for(;;)
  {
    /* Run control loop */
    Flight_ControlLoop();
    
    /* GPS data is updated via interrupt - copy to flight state */
    GPS_GetData(&hgps, &flight_state.gps);
    
    /* Wait for next period */
    last_tick += loop_period_ms;
    osDelayUntil(last_tick);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StarttelemetryTask */
/**
* @brief Function implementing the telemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StarttelemetryTask */
void StarttelemetryTask(void *argument)
{
  /* USER CODE BEGIN StarttelemetryTask */
  /* Telemetry pipeline: send latest flight state over LoRa */
  for(;;)
  {
    /* Send telemetry if ready */
    if (Telemetry_ReadyToSend(&htelemetry)) {
      float roll_out, pitch_out, yaw_out;
      FlightPID_GetOutputs(&flight_pid, &roll_out, &pitch_out, &yaw_out);
      uint8_t telem_roll = (uint8_t)((roll_out + 1.0f) * 127.5f);
      uint8_t telem_pitch = (uint8_t)((pitch_out + 1.0f) * 127.5f);
      uint8_t telem_yaw = (uint8_t)((yaw_out + 1.0f) * 127.5f);
      uint8_t telem_throttle = (uint8_t)(throttle_command * 255.0f);
      Telemetry_BuildPacket(&htelemetry, &flight_state,
                            telem_roll, telem_pitch, telem_yaw, telem_throttle);
      Telemetry_Send(&htelemetry);
    }
    Telemetry_Update(&htelemetry);
    osDelay(10); // 100Hz loop
  }
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StarttelemetryTask */
}

/* USER CODE BEGIN Header_StartsensorsTask */
/**
* @brief Function implementing the sensorsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartsensorsTask */
void StartsensorsTask(void *argument)
{
  /* USER CODE BEGIN StartsensorsTask */
  /* Sensor polling: update IMU, baro, power, GPS */
  for(;;)
  {
    /* IMU update */
    if (MPU6050_ReadAll(&hmpu, &flight_state.imu) == HAL_OK) {
      flight_state.last_imu_update = HAL_GetTick();
      Kalman_Update(&kalman, &flight_state.imu.accel, &flight_state.imu.gyro, 0.01f);
      Kalman_GetOrientation(&kalman, &flight_state.orientation);
    }
    /* Barometer update */
    MS5611_ReadBlocking(&hbaro, &flight_state.baro);
    /* Power sensor update */
    PowerSensor_Read(&hpower, &flight_state.power);
    /* GPS update (if needed) */
    GPS_GetData(&hgps, &flight_state.gps);
    osDelay(10); // 100Hz loop
  }
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartsensorsTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
