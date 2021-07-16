/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define YL_IN1_Pin GPIO_PIN_12
#define YL_IN1_GPIO_Port GPIOB
#define YL_IN2_Pin GPIO_PIN_13
#define YL_IN2_GPIO_Port GPIOB
#define YR_IN1_Pin GPIO_PIN_14
#define YR_IN1_GPIO_Port GPIOB
#define YR_IN2_Pin GPIO_PIN_15
#define YR_IN2_GPIO_Port GPIOB
#define XR_IN2_Pin GPIO_PIN_11
#define XR_IN2_GPIO_Port GPIOG
#define XR_IN1_Pin GPIO_PIN_12
#define XR_IN1_GPIO_Port GPIOG
#define XL_IN2_Pin GPIO_PIN_13
#define XL_IN2_GPIO_Port GPIOG
#define XL_IN1_Pin GPIO_PIN_14
#define XL_IN1_GPIO_Port GPIOG
#define MPU_INT_Pin GPIO_PIN_5
#define MPU_INT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
