/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define LED_0_Pin GPIO_PIN_15
#define LED_0_GPIO_Port GPIOC
#define COM_TX_Pin GPIO_PIN_0
#define COM_TX_GPIO_Port GPIOA
#define COM_RX_Pin GPIO_PIN_1
#define COM_RX_GPIO_Port GPIOA
#define ADC_BAT_Pin GPIO_PIN_2
#define ADC_BAT_GPIO_Port GPIOA
#define ADC_4V_Pin GPIO_PIN_3
#define ADC_4V_GPIO_Port GPIOA
#define ADC_3_3V_Pin GPIO_PIN_4
#define ADC_3_3V_GPIO_Port GPIOA
#define ROPE_CUT_Pin GPIO_PIN_5
#define ROPE_CUT_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOA
#define S1_CAM_Pin GPIO_PIN_7
#define S1_CAM_GPIO_Port GPIOA
#define S0_CAM_Pin GPIO_PIN_0
#define S0_CAM_GPIO_Port GPIOB
#define MUX_EN_1_Pin GPIO_PIN_1
#define MUX_EN_1_GPIO_Port GPIOB
#define S2_SENS_Pin GPIO_PIN_8
#define S2_SENS_GPIO_Port GPIOA
#define S1_SENS_Pin GPIO_PIN_6
#define S1_SENS_GPIO_Port GPIOC
#define S0_SENS_Pin GPIO_PIN_11
#define S0_SENS_GPIO_Port GPIOA
#define MUX_EN_0_Pin GPIO_PIN_12
#define MUX_EN_0_GPIO_Port GPIOA
#define CAM_HB_1_Pin GPIO_PIN_15
#define CAM_HB_1_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_6
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_7
#define DEBUG_RX_GPIO_Port GPIOB
#define CAM_HB_0_Pin GPIO_PIN_8
#define CAM_HB_0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
