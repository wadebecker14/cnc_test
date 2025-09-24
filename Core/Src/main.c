/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Helper: set direction */
void set_dir(GPIO_TypeDef *port, uint16_t pin, uint8_t dir)
{
    HAL_GPIO_WritePin(port, pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Start step pulses on chosen channel */
void start_steps(uint8_t axis, uint32_t freq_hz)
{
//    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq(); // e.g. 32 MHz
    uint32_t prescaler = 31;                     // 1 MHz tick
    uint32_t arr = (1000000 / freq_hz) - 1;

    htim2.Instance->PSC = prescaler;
    htim2.Instance->ARR = arr;
    htim2.Instance->CCR1 = arr / 2;
    htim2.Instance->CCR2 = arr / 2;
    htim2.Instance->CCR3 = arr / 2;
    htim2.Instance->CCR4 = arr / 2;

    switch(axis) {
        case 0: HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); break; // X
        case 1: HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); break; // Y
        case 2: HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); break; // Z
        case 3: HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); break; // A
    }
}

/* Stop step pulses */
void stop_steps(uint8_t axis) {
    switch(axis) {
        case 0: HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); break; // X
        case 1: HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); break; // Y
        case 2: HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); break; // Z
        case 3: HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); break; // A
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buf[20];
  uint8_t bufLen = 0;
  HAL_TIM_StateTypeDef done = 0;
  uint32_t steps = 2000;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Example: Move X axis forward at 1 kHz for ~2s
//	  set_dir(X_DIR_Port, X_DIR_Pin, 1);   // forward
//	  start_steps(0, 1000);                // 1 kHz pulses
//	  HAL_Delay(2000);
//	  stop_steps(0);
//	  sprintf(buf,"X forward\n");
//	  bufLen = sizeof(buf);
//	  HAL_UART_Transmit(&huart1, buf, bufLen, 100);
//
//	  HAL_Delay(500);
//
//	  // Move back at 2 kHz for ~2s
//	  set_dir(X_DIR_Port, X_DIR_Pin, 0);   // reverse
//	  start_steps(0, 2000);                // 2 kHz pulses
//	  HAL_Delay(2000);
//	  stop_steps(0);
//	  sprintf(buf,"X reverse\n");
//	  bufLen = sizeof(buf);
//	  HAL_UART_Transmit(&huart1, buf, bufLen, 100);
//
//	  HAL_Delay(500);

	  // Example: Move Y axis forward at 1 kHz for ~2s
	  set_dir(Y_DIR_Port, Y_DIR_Pin, 1);   // forward
	  start_steps(1, 500);                // 1 kHz pulses
	  HAL_Delay(10000);
	  stop_steps(1);

	  HAL_Delay(500);

	  set_dir(Y_DIR_Port, Y_DIR_Pin, 0);   // forward
	  start_steps(1, 500);                // 1 kHz pulses
	  HAL_Delay(10000);
	  stop_steps(1);

	  HAL_Delay(500);

	  // Example: Move Z axis forward at 1 kHz for ~2s
//	  set_dir(Z_DIR_Port, Z_DIR_Pin, 1);   // forward
//	  start_steps(2, 500);                // 1 kHz pulses
//	  HAL_Delay(10000);
//	  stop_steps(2);
//
//	  HAL_Delay(500);
//
//	  set_dir(Z_DIR_Port, Z_DIR_Pin, 0);   // forward
//	  start_steps(2, 500);                // 1 kHz pulses
//	  HAL_Delay(10000);
//	  stop_steps(2);
//
//	  HAL_Delay(500);

	  // Example: Move A axis forward at 1 kHz for ~2s
//	  set_dir(A_DIR_Port, A_DIR_Pin, 1);   // forward
//	  start_steps(3, 500);                // 1 kHz pulses
//	  HAL_Delay(10000);
//	  stop_steps(3);
//
//	  HAL_Delay(500);
//
//	  set_dir(A_DIR_Port, A_DIR_Pin, 0);   // forward
//	  start_steps(3, 500);                // 1 kHz pulses
//	  HAL_Delay(10000);
//	  stop_steps(3);
//
//	  HAL_Delay(500);
//
//	  // Move back at 2 kHz for ~2s
//	  set_dir(A_DIR_Port, A_DIR_Pin, 0);   // reverse
//	  start_steps(3, 4000);                // 2 kHz pulses
//	  done = HAL_TIM_PWM_GetState(&htim2);
//	  while(done == HAL_TIM_STATE_BUSY)
//	  {
//		  done = HAL_TIM_PWM_GetState(&htim2);
//
//		  sprintf(buf,"done + %d\r\n", done);
//		  bufLen = sizeof(buf);
//		  HAL_UART_Transmit(&huart1, buf, bufLen, 100);
//	  }
//	  HAL_Delay(4000);
//	  stop_steps(3);
//	  sprintf(buf,"A reverse\n");
//	  bufLen = sizeof(buf);
//	  HAL_UART_Transmit(&huart1, buf, bufLen, 100);
//
//	  HAL_Delay(500);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 32-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, X_DIR_Pin|Y_DIR_Pin|Z_DIR_Pin|ENA9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_Pin|A_DIR_Pin|Y_DIRB10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : X_DIR_Pin Y_DIR_Pin Z_DIR_Pin */
  GPIO_InitStruct.Pin = X_DIR_Pin|Y_DIR_Pin|Z_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin Y_DIRB10_Pin */
  GPIO_InitStruct.Pin = EN_Pin|Y_DIRB10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : A_DIR_Pin */
  GPIO_InitStruct.Pin = A_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENA9_Pin */
  GPIO_InitStruct.Pin = ENA9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENA9_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  	/*Configure GPIO pins : X_DIR_Pin Y_DIR_Pin Z_DIR_Pin */
	GPIO_InitStruct.Pin = X_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  	/*Configure GPIO pins : X_DIR_Pin Y_DIR_Pin Z_DIR_Pin */
	GPIO_InitStruct.Pin = Y_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  	/*Configure GPIO pins : X_DIR_Pin Y_DIR_Pin Z_DIR_Pin */
	GPIO_InitStruct.Pin = Z_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : A_DIR_Pin */
	GPIO_InitStruct.Pin = A_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(A_DIR_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ENA9_Pin */
	GPIO_InitStruct.Pin = EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(EN_Port, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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
