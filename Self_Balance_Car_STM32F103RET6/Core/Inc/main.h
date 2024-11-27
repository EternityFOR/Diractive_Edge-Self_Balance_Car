/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void Timer_Start(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void msg_rx(void *argument);
void Motor_update(void *argument);
void openRadarEngineeringModeUSART2(void);
void ReadPIDParamsFromFlash(float *Balance_Kp, float *Balance_Ki, float *Balance_Kd, float *Velocity_Kp, float *Velocity_Ki, float *Velocity_Kd, float *Balance_Shift);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M2_EN1_Pin GPIO_PIN_0
#define M2_EN1_GPIO_Port GPIOA
#define M2_EN2_Pin GPIO_PIN_1
#define M2_EN2_GPIO_Port GPIOA
#define M2_IB2_Pin GPIO_PIN_6
#define M2_IB2_GPIO_Port GPIOC
#define M2_IA2_Pin GPIO_PIN_7
#define M2_IA2_GPIO_Port GPIOC
#define M3_IB3_Pin GPIO_PIN_8
#define M3_IB3_GPIO_Port GPIOC
#define M3_IA3_Pin GPIO_PIN_9
#define M3_IA3_GPIO_Port GPIOC
#define M4_IA4_Pin GPIO_PIN_8
#define M4_IA4_GPIO_Port GPIOA
#define M4_IB4_Pin GPIO_PIN_9
#define M4_IB4_GPIO_Port GPIOA
#define M1_IA1_Pin GPIO_PIN_10
#define M1_IA1_GPIO_Port GPIOA
#define M1_IB1_Pin GPIO_PIN_11
#define M1_IB1_GPIO_Port GPIOA
#define M3_EN1_Pin GPIO_PIN_15
#define M3_EN1_GPIO_Port GPIOA
#define Running_Mode_Pin GPIO_PIN_10
#define Running_Mode_GPIO_Port GPIOC
#define Mode_Detection_Pin GPIO_PIN_11
#define Mode_Detection_GPIO_Port GPIOC
#define Charging_Mode_Pin GPIO_PIN_12
#define Charging_Mode_GPIO_Port GPIOC
#define M3_EN2_Pin GPIO_PIN_3
#define M3_EN2_GPIO_Port GPIOB
#define M4_EN1_Pin GPIO_PIN_4
#define M4_EN1_GPIO_Port GPIOB
#define M4_EN2_Pin GPIO_PIN_5
#define M4_EN2_GPIO_Port GPIOB
#define M1_EN1_Pin GPIO_PIN_6
#define M1_EN1_GPIO_Port GPIOB
#define M1_EN2_Pin GPIO_PIN_7
#define M1_EN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void UART_IRQHandler_IDLE(UART_HandleTypeDef *huart);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
