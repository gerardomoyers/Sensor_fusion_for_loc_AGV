/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include <time.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

///// UWB Variables
float UWBx;
float UWBy;
float speedUWBx;
float speedUWBy;
float UWBx_previous;
float UWBy_previous;


//// IMU Variables
float accel_x_g;
float accel_y_g;
float accel_z_g;
float gyro_x_rad;
float gyro_y_rad;
float gyro_z_rad;
float accel_x_offset;
float accel_y_offset;
float accel_z_offset;
float gyro_x_offset;
float gyro_y_offset;
float gyro_z_offset;
float accel_x_g2;
float accel_x_g2_previous;
float accel_y_g2;
float accel_z_g2;
float gyro_x_rad2;
float gyro_y_rad2;
float gyro_z_rad2;
float gyro_conv_x;
float gyro_conv_y;
float gyro_conv_z;

float mean_acc_x;
float mean_acc_y;
float mean_acc_z;
float mean_gyro_x;
float mean_gyro_y;
float mean_gyro_z;
_Bool moving;
_Bool turning;
uint8_t Mean;

float distx_rec;
float anglez_rec;

// PID variables
float p_error;
float i_error;
float d_error;
float previous_e;
float Kp;
float Ki;
float Kd;


// Control vehicle variable
_Bool offset;

float angledes;
float anglezactual;
float angle_accumulated;
float disth;
float anglez;
uint8_t heading;

uint8_t USlr;

char Direction[5];


// Radar Variables


typedef struct __Radar_Objects{
	float speed;
	float peak;
	float xobj;
	float yobj;
	float zobj;
}Radar_Objects;

typedef struct __Radar_Clusters{
	float xclu;
	float yclu;
	float xsizeclu;
	float ysizeclu;
}Radar_Clusters;


Radar_Objects Objects_radar[200];
Radar_Objects Objects_radar_previous[200];
Radar_Objects Objects_radar_previous_2[200];
Radar_Clusters Clusters_radar[24];
float Radarx;
float Radary;
float speedradarx;
float speedradary;

uint16_t numObj;
uint16_t numClu;
uint16_t numObj_prev;
uint16_t numClu_prev;
uint16_t numObj_prev2;
uint16_t numClu_prev2;
uint16_t counter_previous_update;
uint16_t update_factor;

_Bool frontback;

float moveallx; // move of the vehicle in x
float moveally; // move of the vehicle in y
float speedall;
uint8_t meanmove;
float gap;

float moveobjx; // move on realworld x of the prev obj
float moveobjy; // move on realworld y of the prev obj
float speedobj;
uint8_t objinrange;

float moveobjx2; // move on realworld x of the prev 2 obj
float moveobjy2; // move on realworld y of the prev 2 obj
uint8_t objinrange2;



// Map Variables

typedef struct __Map_Objects{
	float xLow;
	float yLow;
	float zLow;
	float xTop;
	float yTop;
	float zTop;

}Map_Objects;


// times for each sensor
float start_imu;
float deltatime_imu;
float start_EKF, deltatime_EKF;


//Sensor fusion var
float weight_UWBx;
float weight_UWBy;
float weight_radarx;
float weight_radary;
float weight_speedUWBx;
float weight_speedUWBy;
float weight_speedradarx;
float weight_speedradary;
//float Matrix_A[100];
//float Matrix_B[100];
//float Matrix_result [100];
float jacobian_F[4][4]; // Jacobian states df/dx
float jacobian_H[4][4]; // Jacobian measurements dh/dx
float covariance_P[4][4]; // Predicted covariance F P F' + Q   uncertainty
float standar_dev_Q[4][4]; // sigma^2 input
float standar_dev_R[4][4]; // sigma^2 measurements
float states_X [4];
float states_X_previous [4];
float measurementes_Z [4];
float measurementes_Z_previous [4];
uint16_t UWB_sampling;



uint32_t start;
uint32_t stop;


float angle_accumulated2;
uint16_t actual_freq;
uint16_t prev_freq;
uint16_t actual_freq_radar;
uint16_t actual_freq_radar2;
uint16_t prev_freq_radar;
uint16_t test_straight;
uint32_t count_ch_freq;
uint32_t count_ch_freq_radar;









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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_OverCurrent_Pin GPIO_PIN_6
#define USB_OverCurrent_GPIO_Port GPIOG
#define STLINK_TX_Pin GPIO_PIN_7
#define STLINK_TX_GPIO_Port GPIOG
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define Motor_Right_GPIO GPIOB
#define Motor_Left_GPIO GPIOC
#define Motor_Right_Pin GPIO_PIN_6
#define Motor_Left_Pin GPIO_PIN_7
#define false 0
#define true 1
#define USleft 1
#define USright 2

#define IMU_angle_acc 5
#define Motor_low_limit 20




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
