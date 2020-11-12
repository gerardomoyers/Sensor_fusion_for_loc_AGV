/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uarts.h"
#include "Sensors_functions.h"
#include <math.h>

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for Fusion_Localiza */
osThreadId_t Fusion_LocalizaHandle;
const osThreadAttr_t Fusion_Localiza_attributes = {
  .name = "Fusion_Localiza",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for Data_sensors_Rx */
osThreadId_t Data_sensors_RxHandle;
const osThreadAttr_t Data_sensors_Rx_attributes = {
  .name = "Data_sensors_Rx",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for Control_Motors */
osThreadId_t Control_MotorsHandle;
const osThreadAttr_t Control_Motors_attributes = {
  .name = "Control_Motors",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM7_Init(void);
void StartFusion(void *argument);
void StartRxSensors(void *argument);
void Start_Motors_Ctrl(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PC Variables
_Bool Initial_pos = false;

uint16_t saveUWB = 0;
uint16_t tempsaveUWB = 0;
_Bool SUW = false;

uint16_t saveIMU = 0;
uint16_t tempsaveIMU = 0;
_Bool SIMU = false;

uint16_t saveRadar = 0;
uint16_t tempsaveRadar = 0;
_Bool SRadar = false;

uint16_t saveEKF = 0;
uint16_t tempsaveEKF = 0;
_Bool SEKF = false;

// UART Variables

	static char bufferIMU[LINEMAX +1];
	static char bufferPC[LINEMAX +1];
	static char bufferUWB[LINEMAX +1];
	static char bufferRadar[LINEMAX +1];
	static char bufferUS[LINEMAX +1];
	static char bufferLidar[LINEMAX +1];

uint16_t index_buffer = 0;
_Bool received[6] = {false, false, false, false, false ,false};


// UWB Variables



// Movement Variables (Motors Control)
int16_t speed = Motor_low_limit;
int16_t increase_speed = 0;

uint16_t distance_US = 10;
uint16_t move = 0;

float steer = 0;

float distx = 1;
float disty = 1;
uint16_t offsetcount = 0;

char data[15];


float v1  = 0;
float v2  = 0;
float v3  = 0;
float v4  = 0;
float v5  = 0;
float v6  = 0;
float v7  = 0;
float v8  = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	 // IMU variables

	 accel_x_offset = 0;
	 accel_y_offset = 0;
	 accel_z_offset = 0;
	 gyro_x_offset = 0;
	 gyro_y_offset = 0;
	 gyro_z_offset = 0;
	 mean_acc_x = 0;
	 mean_acc_y = 0;
	 mean_acc_z = 0;
	 mean_gyro_x = 0;
	 mean_gyro_y = 0;
	 mean_gyro_z = 0;
	 gyro_z_rad2 = 0;
	 accel_x_g2 = 0;
	 accel_y_g2 = 0;
	 moving = false;
	 turning = false;
	 Mean = 0;
	 USlr = 0;
	 heading = 1;
	 deltatime_imu = 0;
	 weight_UWBx = 0.3;
	 weight_UWBy = 0.7;
	 weight_radarx = 0.7;
	 weight_radary = 0.3;
	 weight_speedUWBx = 0.4;
	 weight_speedUWBy = 0.4;
	 weight_speedradarx = 0.6;
	 weight_speedradary = 0.6;

	 //UWB Variables
	 UWBx = 0;
	 UWBy = 0;
	 UWBx_previous = 0;
	 UWBx_previous = 0;


	 // Radar Variables
	 Radarx = 0;
	 Radary = 0;
	 numClu = 0;
	 numObj = 0;
	 frontback = true;
	 counter_previous_update = 0;

	 anglezactual = 0;
	 angle_accumulated = 0;
	 angle_accumulated2 = 0;

	 offset = false;
	 disth = 1;
	 anglez = 90.0;

	 // EKF Variables
	 UWB_sampling = true;
	 actual_freq = 1;
	 test_straight = 0;

	 strcpy(Direction,"IDLE");

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
  MX_LPUART1_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	  HAL_NVIC_SetPriority(LPUART1_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(LPUART1_IRQn);
	   HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(USART1_IRQn);
	   HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(USART2_IRQn);
	  HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(USART3_IRQn);
	  HAL_NVIC_SetPriority(UART4_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(UART4_IRQn);
	  HAL_NVIC_SetPriority(UART5_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(UART5_IRQn);


	  IMU_star_com();


	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 1);
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	  LPUART1->CR1 |= USART_CR1_RXNEIE; // Enable Interrupt
	  USART1->CR1 |= USART_CR1_RXNEIE; // Enable Interrupt
	  USART2->CR1 |= USART_CR1_RXNEIE; // Enable Interrupt
	  USART3->CR1 |= USART_CR1_RXNEIE; // Enable Interrupt
	  UART4->CR1 |= USART_CR1_RXNEIE; // Enable Interrupt
	  UART5->CR1 |= USART_CR1_RXNEIE; // Enable Interrupt



	  HAL_UART_Transmit(&huart2,(uint8_t*)"\r", strlen("\r"),1000);
	  osDelay(1);
	  HAL_UART_Transmit(&huart2,(uint8_t*)"\r", strlen("\r"),1000);

	  HAL_UART_Transmit(&hlpuart1,(uint8_t*)"Si funciono", strlen("Si funciono"),1000);

	  TIM2->CCR1 = 120/100*50;// delay 1 micro
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  TIM7->CR1 = 0x01; // CEN(Counter ENable)='1'

	  HCSR4_Init(received);

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
  /* creation of Fusion_Localiza */
  Fusion_LocalizaHandle = osThreadNew(StartFusion, NULL, &Fusion_Localiza_attributes);

  /* creation of Data_sensors_Rx */
  Data_sensors_RxHandle = osThreadNew(StartRxSensors, NULL, &Data_sensors_Rx_attributes);

  /* creation of Control_Motors */
  Control_MotorsHandle = osThreadNew(Start_Motors_Ctrl, NULL, &Control_Motors_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B03FDB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 204800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 120;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 59955;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PF14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PG5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*UART Receive data with interrupts no callbacks */
void QueueBuffer( char data[LINEMAX + 1],  uint32_t index_data, uint8_t index_uart){
	switch (index_uart){
	case 0:
		strncpy(bufferLidar, data, index_data);
		break;
	case 1:
		strncpy(bufferUWB, data, index_data);
		break;
	case 2:
		memcpy(bufferRadar,data, index_data);
	//	strncpy(bufferRadar, data, index_data);
		break;
	case 3:
		strncpy(bufferIMU , data, index_data);
		break;
	case 4:
		strncpy(bufferUS, data, index_data);
		break;
	case 5:
		strncpy(bufferPC, data, index_data);
		break;
	}

	index_buffer = index_data;
	received[index_uart] = true;
}

void DelayMicro(uint32_t delay){
	uint32_t wait = 0;
	while (wait < delay){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
			wait++;
	}
}


/// interrupt IMU Sujays
void EXTI15_10_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
//	mpu6050_read_sensors();
	received[3] = true;
}




// C program for implementation of ftoa()

// Reverses a string 'str' of length 'len'
void reverse(char* str, int len, int sign)
{
	int i = 0, j = len - 1, temp;
	if(sign < 0)i++;
	while (i < j) {
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++;
		j--;
	}
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d, int sign)
{
	int i = 0;
	if(sign < 0)str[i++]='-';
	while (x) {
		str[i++] = (x % 10) + '0';
		x = x / 10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
		str[i++] = '0';

	reverse(str, i,sign);
	str[i] = '\0';
	return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
	// Extract integer part
	float nabs = fabs(n);
	int ipart = (int)nabs;

	// Extract floating part
	float fpart = nabs - (float)ipart;

	// convert integer part to string
	int i = intToStr(ipart, res, 0, (n/nabs));

	// check for display option after point
	if (afterpoint != 0) {
		if(n/nabs < 0){
			res[i] = '.'; // add dot
		}else{
			res[i] = '.'; // add dot
		}

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter
		// is needed to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		intToStr((int)fpart, res + i + 1, afterpoint,0);
	}
}

void PID (void){
	PID_UpdateError(angledes-anglezactual);
	steer = PID_TotalError();

	uint8_t tempright = 1;
	uint8_t templeft = 1;
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)){
		  if (speed + (int16_t)steer > Motor_low_limit && speed + (int16_t)steer < 100){
			  tempright = speed + (int16_t)steer;
		  }else{
				 tempright = (speed + (int16_t)steer < Motor_low_limit) ? Motor_low_limit : 100;
		  }
		  if(speed - (int16_t)steer > Motor_low_limit && speed - (int16_t)steer < 100){
			  templeft = speed - (int16_t)steer;
		  }else{
			  templeft = (speed - (int16_t)steer < Motor_low_limit) ? Motor_low_limit : 100;
		  }
	  }else{
		 tempright = (speed - (int16_t)steer < Motor_low_limit) ? Motor_low_limit : speed - (int16_t)steer;
		 templeft = (speed + (int16_t)steer > 100) ? 100 : speed + (int16_t)steer;
	  }
		TIM4->CCR3 = 59955/100*tempright;// motor right
		TIM4->CCR4 = 59955/100*templeft;// motor left
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	//PID_Init(0.75, 15e-5, 0.115);//speed

}


void ReceiveData_PC (char buff[LINEMAX + 1]){
	uint8_t tempheading = heading;
	_Bool Aheadingchange = false;
	_Bool Dheadingchange = false;
	for (int i = 0; i < index_buffer; i++){
		  if(buff[i] == 'i'){
			  increase_speed += 5;
		  }else if(buff[i] == 'k'){
			  increase_speed -= 5;
		  }else if(buff[i] == 'w'){  // foward
			  strcpy(Direction,"move");
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			  distx = 1;
			  disty = 0;
			  frontback = true;
		  }else if(buff[i] == 's'){  // backward
			  strcpy(Direction,"move");
			  frontback = false;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
			  distx = 1;
			  disty = 0;
		  }else if(buff[i] == 'a'){  // left
			  strcpy(Direction,"move");
			  frontback = true;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			  distx = 0;
			  disty = -0.001;
			  if(tempheading < 4){
				  tempheading = (Aheadingchange) ? tempheading : tempheading + 1;
			  } else{
				  tempheading = (Aheadingchange) ?  tempheading : 1;
			  }
			  Aheadingchange = true;
			  Dheadingchange = false;
		  }else if(buff[i] == 'd'){  // right
			  strcpy(Direction,"move");
			  frontback = true;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			  distx = 0;
			  disty = 0.001;
			  if(tempheading > 1){
				  tempheading = (Dheadingchange) ? tempheading : tempheading - 1;
			  } else{
				  tempheading = (Dheadingchange) ?  tempheading : 4;
			  }
			  Aheadingchange = false;
			  Dheadingchange = true;


		  }else if(buff[i] == '1'){
			  strcpy(Direction,"move");
			  frontback = true;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			  distx = 1;
			  disty = 1;
		  }else if(buff[i] == '2'){
			  strcpy(Direction,"move");
			  frontback = true;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			  distx = 1;
			  disty = -1;
		  }else if(buff[i] == '3'){
			  strcpy(Direction,"move");
			  frontback = true;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			  distx = 2;
			  disty = 1;
		  }else if(buff[i] == '4'){
			  strcpy(Direction,"move");
			  frontback = true;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			  distx = 2;
			  disty = -1;
		  }else if(buff[i] == '5'){
			  strcpy(Direction,"move");
			  frontback = true;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			  distx = 2;
			  disty = 0;
		  }else if(buff[i] == '6'){
			  strcpy(Direction,"move");
			  frontback = true;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			  distx = 13;
			  disty = 0;
		  }else if(buff[i] == 'o'){
			  offset = false;
			  if(buff[i+1] == 'o'){
				  angle_accumulated = 0;
			  }else if(buff[i+1] == 'p'){
				  angle_accumulated = -180;
			  }
		  }else if(buff[i] == 'u'){
			  HAL_UART_Transmit(&huart2,(uint8_t*)"\r", strlen("\r"),1000);
			  //osDelay(1);
			  HAL_UART_Transmit(&huart2,(uint8_t*)"\r", strlen("\r"),1000);
			  if(buff[i+1] == 'r'){
				  HAL_UART_Transmit(&huart2,(uint8_t*)"aurs 10 20\r", strlen("aurs 10 20\r"),1000);
			  }
		  }else if(buff[i] == 'q'){
			  tempheading = heading;
			  Aheadingchange = false;
			  Dheadingchange = false;
			  strcpy(Direction,"quit");
		  }else if(buff[i] == 'c'){
			  if(buff[i+1] == 'v'){
				  SUW = true;
				  tempsaveUWB = saveUWB;
			  }else if(buff[i+1] == 'b'){
				  SIMU = true;
				  tempsaveIMU = saveIMU;
			  }else if(buff[i+1] == 'n'){
				  SRadar = true;
				  tempsaveRadar = saveRadar;
			  }else{
				  SEKF = true;
				  tempsaveEKF = saveEKF;
			  }
		  }else if(buff[i] == 'f'){
			  Initial_pos = false;

		  }else {
/*
			  HAL_UART_Transmit(&huart2,(uint8_t*)"\r", strlen("\r"),1000);
			  HAL_UART_Transmit(&huart2,(uint8_t*)"\r", strlen("\r"),1000);
			  HAL_UART_Transmit(&huart2,(uint8_t*)"\r", strlen("\r"),1000);*/

		  }
	  }
	heading = tempheading;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartFusion */
/**
  * @brief  Function implementing the Fusion_Localiza thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartFusion */
void StartFusion(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  //distance_US = HCSR4_Meas(distance_US, received);
	  if(!Initial_pos){
			Init_localization(7,2.5,0,0);
			Initial_pos = true;
			measurementes_Z[0] = Radarx;
			measurementes_Z[1] = Radary;
			measurementes_Z[2] = speedradarx;
			measurementes_Z[3] = speedradary;
			actual_freq = 1;
			prev_freq = 1;
			actual_freq_radar = 1;
			actual_freq_radar2 = 1;
			Initial_pos = true;

	  }else if(Initial_pos){

		  if(received > 0){
			  if(SEKF && moving){
					SEKF = (saveEKF >= tempsaveEKF + 9000) ? false : true;
					saveEKF++;

					intToStr(actual_freq, data, 0, 1);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"f   ", strlen("f   "), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					intToStr(actual_freq_radar, data, 0, 1);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"fr   ", strlen("fr   "), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					ftoa(states_X[0], data, 4);
			//		HAL_UART_Transmit(&hlpuart1, (uint8_t *)"EKF X:", strlen("EKF X:"	), 1000); //m
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					ftoa(states_X[1], data, 4);
			//		HAL_UART_Transmit(&hlpuart1, (uint8_t *)"EKF Y:", strlen("EKF Y:"	), 1000);  //m
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);


	/*				ftoa(states_X[2], data, 4);
			//		HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Speed X:", strlen("Speed X:"	), 1000); // m/s
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					ftoa(states_X[3], data, 4);
			//		HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Speed Y:", strlen("Speed Y:"	), 1000); // m/s
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"\r\n", strlen("\r\n"), 1000);*/

			  }

			  if (turning){
						memcpy(states_X_previous,states_X,sizeof(states_X));
						memcpy(measurementes_Z_previous,measurementes_Z,sizeof(measurementes_Z));
						/* Random gaussian noise measurements*/
						int dummyv = TIM7 -> CNT;
						v1  = dummyv % 1692;
						v2  = dummyv % 908;
						v3  = dummyv % 5822;
						v4  = dummyv % 4650;
						v5  = dummyv % 5376;
						v6  = dummyv % 2330;
						v7  = dummyv % 583;
						v8  = dummyv % 527;

						v1 = v1/10000 - 0.0846;
						v2 = v2/10000 - 0.0454;
						v3 = v3/10000 - 0.2911;
						v4 = v4/10000 - 0.2325;
						v5 = v5/10000 - 0.2670;//0.2688;
						v6 = v6/10000 - 0.1165;
						v7 = v7/10000 - 0.0291;
						v8 = v8/10000 - 0.0263;


						measurementes_Z[0] = (UWBx + v1) * weight_UWBx + (Radarx + v5) * weight_radarx;
						measurementes_Z[1] = (UWBy + v2) * weight_UWBy + (Radary + v6) * weight_radary;
						measurementes_Z[2] = (speedUWBx + v3) * weight_speedUWBx + (speedradarx + v7) * weight_speedradarx;
						measurementes_Z[3] = (speedUWBy + v4) * weight_speedUWBy + (speedradary + v8) * weight_speedradary;


						EKF_prediction();
						EKF_update();

			  }else {

						memcpy(states_X_previous,states_X,sizeof(states_X));
						memcpy(measurementes_Z_previous,measurementes_Z,sizeof(measurementes_Z));
						/* Random gaussian noise measurements */
						int dummyv = TIM7 -> CNT;
						v1  = dummyv % 1692;
						v2  = dummyv % 908;
						v3  = dummyv % 5822;
						v4  = dummyv % 4650;
						v5  = dummyv % 5376;
						v6  = dummyv % 2330;
						v7  = dummyv % 583;
						v8  = dummyv % 527;

						v1 = v1/10000 - 0.0846;
						v2 = v2/10000 - 0.0454;
						v3 = v3/10000 - 0.2911;
						v4 = v4/10000 - 0.2325;
						v5 = v5/10000 - 0.2670;//0.2688;
						v6 = v6/10000 - 0.1165;
						v7 = v7/10000 - 0.0291;
						v8 = v8/10000 - 0.0263;


						measurementes_Z[0] = (UWBx + v1) * weight_UWBx + (Radarx + v5) * weight_radarx;
						measurementes_Z[1] = (UWBy + v2) * weight_UWBy + (Radary + v6) * weight_radary;
						measurementes_Z[2] = (speedUWBx + v3) * weight_speedUWBx + (speedradarx + v7) * weight_speedradarx;
						measurementes_Z[3] = (speedUWBy + v4) * weight_speedUWBy + (speedradary + v8) * weight_speedradary;


						EKF_prediction();
						EKF_update();

			  }

		/*		stop = HAL_GetTick() - start;
				intToStr(stop, data, 0, 1);
				HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
				HAL_UART_Transmit(&hlpuart1, (uint8_t *)" \r\n", strlen(" \r\n"), 1000);

				start = HAL_GetTick();
*/
		  }
	  }




	  //HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartRxSensors */
/**
* @brief Function implementing the Data_sensors_Rx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRxSensors */
void StartRxSensors(void *argument)
{
  /* USER CODE BEGIN StartRxSensors */
	uint16_t counter_imu = 500;
	  float bias_gyro = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(received[3]){  //
			  received[3] = false;
//			  HAL_UART_Transmit(&hlpuart1,(uint8_t*)bufferIMU, index_buffer,1000);

			  	accel_x_g2_previous = accel_x_g2;
				mpu6050_read_sensors();
				offsetcount = IMU_Meas(offsetcount);
				start_imu = HAL_GetTick();
				float delta_angle = (gyro_z_rad2 * deltatime_imu)/10;
			  	if (counter_imu++ > 499 && moving == false){
			  		bias_gyro = delta_angle;
			  		counter_imu = 0;

			  	}
				angle_accumulated += (delta_angle - bias_gyro);
				float UVy = cosf(angle_accumulated);
				float UVx = sinf(angle_accumulated);
				if(angle_accumulated > angle_accumulated2 + 10 || angle_accumulated < angle_accumulated2 - 10 && test_straight == 999){
					actual_freq = (prev_freq == 5 ) ? 1 : 5;
					prev_freq = actual_freq;
					actual_freq_radar = (actual_freq_radar2 == 5 ) ? 1 : 5;
					actual_freq_radar2 = actual_freq_radar;
				}else if(test_straight == 999){
						actual_freq = actual_freq + 1;
						actual_freq = (actual_freq > 10) ? 10 : actual_freq;

					actual_freq = (prev_freq == 1) ? 5 : actual_freq;
					prev_freq = actual_freq;
				}

				if(test_straight++ > 1000){
					angle_accumulated2 = angle_accumulated;
					test_straight=0;
				}


				 	 if(SIMU){
					 	SIMU = (saveIMU >= tempsaveIMU + 100) ? false : true;

						saveIMU++;
						ftoa(angle_accumulated, data, 6);
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Angleacc:", strlen("Angleacc:"	), 1000); // deg
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

						ftoa(accel_x_g2, data, 4);
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Accel X:", strlen("Accel X:"	), 1000);  // m/s^2
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

						ftoa(accel_y_g2, data, 4);
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Accel Y:", strlen("Accel Y:"	), 1000);  //// m/s^2
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

						ftoa(UVx, data, 4);
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Speed X:", strlen("Speed X:"	), 1000);  // m/s
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

						ftoa(UVy, data, 4);
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Speed Z:", strlen("Speed Z:"	), 1000);  // m/s
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
						HAL_UART_Transmit(&hlpuart1, (uint8_t *)" \r\n", strlen(" \r\n"), 1000);

					}


		  } if(received[1]){  // UWB
			  received[1] = false;
	//		  float tempSpeedUWBx;
		//	  float tempSpeedUWBy;

			  if(SUW){
				  SUW = (saveUWB >= tempsaveUWB + 100) ? false : true;

				  	saveUWB++;
					ftoa(UWBx, data, 4);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"UWBx:", strlen("UWBx:"	), 1000);  // m
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					ftoa(UWBy, data, 4);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"UWBy:", strlen("UWBy:"	), 1000);  // m
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					ftoa(speedUWBx, data, 4);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"UWBVx:", strlen("UWBVx:"	), 1000); // m/s
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					ftoa(speedUWBy, data, 4);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)"UWBVy:", strlen("UWBVy:"	), 1000); // m/s
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					HAL_UART_Transmit(&hlpuart1, (uint8_t *)" \r\n", strlen(" \r\n"), 1000);

/*				  HAL_UART_Transmit(&hlpuart1,(uint8_t*)bufferUWB, index_buffer,1000);// debug
				  HAL_UART_Transmit(&hlpuart1,(uint8_t*)"\r", strlen("\r"),1000);
				  */
			  }

			  if(UWB_sampling++ >= 11-actual_freq){
				  UWB_sampling = 1;
			  }else{
					  if(bufferUWB[index_buffer-1] == 'p'){
						  osDelay(1);
						  HAL_UART_Transmit(&huart2,(uint8_t*)"lec\r", strlen("lec\r"),1000);
					  }
					  if (strncmp("POS",bufferUWB,3)== 0){
						  UWBx = atof(bufferUWB+11);
						  int i = 11;
						  while(bufferUWB[i] != ',')i++;
						  UWBy = atof(bufferUWB+i+1);

						  if (isnanf(UWBx) || isnanf(UWBy)){
							  UWBx = UWBx_previous;
							  UWBy = UWBy_previous;
						  }
					}

						speedUWBx = (UWBx-UWBx_previous) * 10;
						speedUWBy = (UWBy-UWBy_previous) * 10;
						UWBx_previous = UWBx;
						UWBy_previous = UWBy;
			}

		  } if(received[2]){  // Radar
			  received[2] = false;

			   numObj = 0;
			   numClu = 0;
			   count_ch_freq_radar++;
			   if (turning == false && prev_freq_radar++ >= actual_freq_radar /*powf(2,actual_freq_radar-1)*/){
				   prev_freq_radar = 1;
				   getdata_Radar(bufferRadar);
				   if(Initial_pos && numObj > 0) getPos_Radar();
			   }

			  if(SRadar /*&& moving*/){
				  SRadar = (saveRadar >= tempsaveRadar + 100) ? false : true;
				  saveRadar++;

				  ftoa(Radarx, data, 4);
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Radarx:", strlen("Radarx:"	), 1000); // m
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

				  ftoa(Radary, data, 4);
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Radary:", strlen("Radary:"	), 1000);  // m
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

				  ftoa(speedradarx, data, 4);
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Speedx:", strlen("Speedx:"	), 1000); // m/s
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

				  ftoa(speedradary, data, 4);
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Speedy:", strlen("Speedy:"	), 1000); // m/s
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"\r\n", strlen("\r\n"), 1000);

		/*		  for(uint16_t temp_numObj = 1; temp_numObj <= numObj; temp_numObj++)
				  {

					  ftoa(Objects_radar[temp_numObj-1].speed, data, 2);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Speed:", strlen("Speed:"	), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					  ftoa(Objects_radar[temp_numObj-1].xobj, data, 2);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Objx:", strlen("Objx:"	), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					  ftoa(Objects_radar[temp_numObj-1].yobj, data, 2);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Objy:", strlen("Objy:"	), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					  ftoa(Objects_radar[temp_numObj-1].zobj, data, 2);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Objz:", strlen("Objz:"	), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"\r\n", strlen("\r\n"), 1000);
				  }
				  for(uint16_t temp_numClu = 1; temp_numClu <= numClu; temp_numClu++)
				  {
					  ftoa(Clusters_radar[temp_numClu-1].xclu, data, 2);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Clux:", strlen("Clux:"	), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					  ftoa(Clusters_radar[temp_numClu-1].yclu, data, 2);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Cluz:", strlen("Cluz:"	), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					  ftoa(Clusters_radar[temp_numClu-1].xclu, data, 2);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Clusizex:", strlen("Clusizex:"	), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"   ", strlen("   "), 1000);

					  ftoa(Clusters_radar[temp_numClu-1].ysizeclu, data, 2);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Clusizey:", strlen("Clusizey:"	), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
					  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"\r\n", strlen("\r\n"), 1000);

				  }
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"\r\n", strlen("\r\n"), 1000);
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"\r\n", strlen("\r\n"), 1000);*/
			  }


		  } if(received[0]){  //
			  received[0] = false;
				  HAL_UART_Transmit(&huart4,(uint8_t*)bufferLidar, index_buffer,1000);
		  } if(received[4]){
		  	  received[4] = false;
//		  	  char dUS[5];
		 // 	  distance_US = HCSR4_Meas(distance_US, received);
		  	  if(distance_US < 10){
		  		/*  itoa(distance_US,dUS,10);
		  		  if(USlr == USleft){
		  			  HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Left   ", strlen("Left   "), 1000);
		  		  }else{
		  			HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Right   ", strlen("Right   "), 1000);
		  		  }
				  HAL_UART_Transmit(&hlpuart1, &dUS, 3, 1000);
				  HAL_UART_Transmit(&hlpuart1,(uint8_t*)"\r", strlen("\r"),1000);
*/
				  //move = 30001;

		  	  }

		  } if(received[5]){  // PC
			  received[5] = false;

			  ReceiveData_PC(bufferPC);
			  disth = sqrt(distx*distx+disty*disty);
			  anglez = acos(distx/disth)*180/3.14159;
			  anglez = (disty == 0) ? anglez : anglez * (disty/fabs(disty));
			  angledes = anglez;
			  anglezactual = 0;
//				PID_Init(1.165,1.e-4,0.115);
			  PID_Init(4,4e-4,0.115);

			 // HAL_UART_Transmit(&hlpuart1,(uint8_t*)bufferPC, index_buffer,1000);

		  }

	osDelay(1);
  }
  /* USER CODE END StartRxSensors */
}

/* USER CODE BEGIN Header_Start_Motors_Ctrl */
/**
* @brief Function implementing the Control_Motors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Motors_Ctrl */
void Start_Motors_Ctrl(void *argument)
{
  /* USER CODE BEGIN Start_Motors_Ctrl */
  /* Infinite loop */

  for(;;)
    {
	  //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));
  	  if(increase_speed != 0){
  	  		  speed = speed + increase_speed;
  	  		  if (speed < 25) speed=25; // lower limit motors work
  	  		  if (speed > 100) speed=100; // safe upper limit for motors
  	  		  increase_speed = 0;

  	  	  }

  	if(strcmp(Direction, "move") == 0){
  			  moving = true;
  			  move++;
  		/*	  float dummytime = HAL_GetTick();
  			ftoa(dummytime, data, 4);
			HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Time start:", strlen("Time start:"	), 1000); // m/s
			HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
			HAL_UART_Transmit(&hlpuart1, (uint8_t *)"\r\n", strlen("\r\n"), 1000);
  			*/  strcpy(Direction,"IDLE");
  		  }else if(strcmp(Direction, "quit") == 0 || move == 0 || /*move > 30000 ||*/
  				  (disth <= 0  && (anglez >= -IMU_angle_acc  || anglez <= IMU_angle_acc) )   ){
  			  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  			  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
  			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
  		/*	  if(move != 0) {
  				  osDelay(1000);
  				  offset = false;
  			  }*/
  			  move = 0;
  			/*  if (moving){
  	  			float dummytime = HAL_GetTick();
  				ftoa(dummytime, data, 4);
  				HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Time stop:", strlen("Time stop:"	), 1000); // m/s
  				HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
  				HAL_UART_Transmit(&hlpuart1, (uint8_t *)"\r\n", strlen("\r\n"), 1000);
  			  }*/
  			  moving = false;
  			  strcpy(Direction,"IDLE");
  		  }else{
  			  PID();

  		  }
      osDelay(1);
    }
  /* USER CODE END Start_Motors_Ctrl */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM8) {
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
