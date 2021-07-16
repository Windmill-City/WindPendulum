/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "WindPendulum.hpp"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setMotorMode(Motor::Mode mode, GPIO_TypeDef *port1, uint16_t pin1, GPIO_TypeDef *port2, u_int16_t pin2)
{
  switch (mode)
  {
  case Motor::Free:
    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
    break;
  case Motor::Brake:
    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET);
    break;
  case Motor::Forward:
    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
    break;
  case Motor::Backward:
    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET);
    break;
  }
}

Motor::Mode getMotorMode(GPIO_TypeDef *port1, uint16_t pin1, GPIO_TypeDef *port2, u_int16_t pin2)
{
  auto pin1State = HAL_GPIO_ReadPin(port1, pin1);
  auto pin2State = HAL_GPIO_ReadPin(port2, pin2);

  //Free or Brake
  if (pin1State == pin2State)
  {
    return pin1State == GPIO_PIN_RESET ? Motor::Free : Motor::Brake;
  }
  else
  {
    //Forward or Backward
    return pin1State == GPIO_PIN_SET ? Motor::Forward : Motor::Backward;
  }
}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  using namespace std::placeholders;
  //平面 XZ
  auto pwmXL = PWMGenerater(&htim2, TIM2->CCR1);
  auto pwmXR = PWMGenerater(&htim2, TIM2->CCR2);
  auto motorXL = Motor(
      &pwmXL,
      //Motor::Config -> minStartupTime, minStopTime, minDuty, minStartupDuty, maxStartupDuty, maxDuty
      {50, 20, 1000, 1000, 12000, 16000},
      std::bind(setMotorMode, _1, XL_IN1_GPIO_Port, XL_IN1_Pin, XL_IN2_GPIO_Port, XL_IN2_Pin),
      std::bind(getMotorMode, XL_IN1_GPIO_Port, XL_IN1_Pin, XL_IN2_GPIO_Port, XL_IN2_Pin));
  auto motorXR = Motor(
      &pwmXR,
      {50, 20, 1000, 1000, 12000, 16000},
      std::bind(setMotorMode, _1, XR_IN1_GPIO_Port, XR_IN1_Pin, XR_IN2_GPIO_Port, XR_IN2_Pin),
      std::bind(getMotorMode, XR_IN1_GPIO_Port, XR_IN1_Pin, XR_IN2_GPIO_Port, XR_IN2_Pin));
  auto motorXZ = MotorController(&motorXL, &motorXR);
  //平面 YZ
  auto pwmYL = PWMGenerater(&htim2, TIM2->CCR3);
  auto pwmYR = PWMGenerater(&htim2, TIM2->CCR4);
  auto motorYL = Motor(
      &pwmYL,
      {50, 20, 1000, 1000, 12000, 16000},
      std::bind(setMotorMode, _1, YL_IN1_GPIO_Port, YL_IN1_Pin, YL_IN2_GPIO_Port, YL_IN2_Pin),
      std::bind(getMotorMode, YL_IN1_GPIO_Port, YL_IN1_Pin, YL_IN2_GPIO_Port, YL_IN2_Pin));
  auto motorYR = Motor(
      &pwmYR,
      {50, 20, 1000, 1000, 12000, 16000},
      std::bind(setMotorMode, _1, YR_IN1_GPIO_Port, YR_IN1_Pin, YR_IN2_GPIO_Port, YR_IN2_Pin),
      std::bind(getMotorMode, YR_IN1_GPIO_Port, YR_IN1_Pin, YR_IN2_GPIO_Port, YR_IN2_Pin));
  auto motorYZ = MotorController(&motorYL, &motorYR);

  //风力摆姿态解算
  auto attrProvider = WindPendulum::AttributeProvider();
  //风力摆核心控制器
  auto windPendulum = WindPendulum(&motorXZ, &motorYZ, {9.8, 0.65}, &attrProvider);

  //启动PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    windPendulum.updateLoop();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
