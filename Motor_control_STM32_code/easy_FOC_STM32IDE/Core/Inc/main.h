/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_FREQUENCY 30000
#define TIM_CLK_MHz 170
#define VBUS_ADC1_1_Pin GPIO_PIN_0
#define VBUS_ADC1_1_GPIO_Port GPIOA
#define U_CURR_ADC1_2_Pin GPIO_PIN_1
#define U_CURR_ADC1_2_GPIO_Port GPIOA
#define Debug_Tx_Pin GPIO_PIN_2
#define Debug_Tx_GPIO_Port GPIOA
#define Debug_Rx_Pin GPIO_PIN_3
#define Debug_Rx_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define PWM_ENCODER_IN_Pin GPIO_PIN_6
#define PWM_ENCODER_IN_GPIO_Port GPIOA
#define W_CURR_ADC2_4_Pin GPIO_PIN_7
#define W_CURR_ADC2_4_GPIO_Port GPIOA
#define ENCODER_ADC_Pin GPIO_PIN_4
#define ENCODER_ADC_GPIO_Port GPIOC
#define V_CURR_ADC1_14_Pin GPIO_PIN_11
#define V_CURR_ADC1_14_GPIO_Port GPIOB
#define U_H_DOWN_Pin GPIO_PIN_13
#define U_H_DOWN_GPIO_Port GPIOB
#define V_H_DOWN_Pin GPIO_PIN_14
#define V_H_DOWN_GPIO_Port GPIOB
#define W_H_DOWN_Pin GPIO_PIN_15
#define W_H_DOWN_GPIO_Port GPIOB
#define ENCODER_SCL_Pin GPIO_PIN_8
#define ENCODER_SCL_GPIO_Port GPIOC
#define ENCODER_SDA_Pin GPIO_PIN_9
#define ENCODER_SDA_GPIO_Port GPIOC
#define U_H_UP_Pin GPIO_PIN_8
#define U_H_UP_GPIO_Port GPIOA
#define V_H_UP_Pin GPIO_PIN_9
#define V_H_UP_GPIO_Port GPIOA
#define W_H_UP_Pin GPIO_PIN_10
#define W_H_UP_GPIO_Port GPIOA
#define TIM1_BRAKE_Pin GPIO_PIN_11
#define TIM1_BRAKE_GPIO_Port GPIOA
#define TEST_Pin GPIO_PIN_10
#define TEST_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
