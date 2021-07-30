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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <usbd_cdc_if.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern USBD_HandleTypeDef hUsbDeviceFS;
#define BUF_SIZE 32768

// Flow control
uint8_t switch_pressed = 0;
uint8_t tim7_overflow = 0;
uint8_t pulse_fired = 0;
uint8_t adc_int = 0;

// Data buffers
uint16_t* buffer0;
uint16_t* buffer1;
uint16_t* cur_buf;
volatile uint8_t buf_ready = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_SwitchToPLL(void);

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
//  char string_buffer[64];
  double pulse_time = 0;
  buffer0 = (uint16_t *)malloc(sizeof(uint16_t) * BUF_SIZE);
  buffer1 = (uint16_t *)malloc(sizeof(uint16_t) * BUF_SIZE);
  cur_buf = buffer0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  for (uint16_t i=0; i<BUF_SIZE; ++i) {
    buffer0[i] = 0;
    buffer1[i] = 0;
  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemClock_SwitchToPLL();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_Base_Start_IT(&htim7);
//    HAL_ADC_Start_IT(&hadc1);
//    HAL_PCD_Start(&hpcd_USB_OTG_FS);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)cur_buf, BUF_SIZE);
//    HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if (switch_pressed) {
//        HAL_UART_Transmit(&huart2, (uint8_t*)"hello ", strlen("hello "), HAL_MAX_DELAY);
//        sprintf(string_buffer, "%d", tim7_overflow);
//        HAL_UART_Transmit(&huart2, string_buffer, strlen(string_buffer), HAL_MAX_DELAY);

//        sprintf(string_buffer, " %d ", __HAL_TIM_GET_COUNTER(&htim7));
//        itoa(__HAL_TIM_GET_COUNTER(&htim7), string_buffer, 10);
//        HAL_UART_Transmit(&huart2, string_buffer, strlen(string_buffer), HAL_MAX_DELAY);

        switch_pressed = 0;
        tim7_overflow = 0;
    }
//    if (tim7_overflow > 16) {
//        sprintf(string_buffer, "%d", tim7_overflow);
//        HAL_UART_Transmit(&huart2, string_buffer, strlen(string_buffer), HAL_MAX_DELAY);
//        HAL_UART_Transmit(&huart2, (uint8_t*)"hello ", strlen("hello "), HAL_MAX_DELAY);
//        HAL_UART_Transmit(&huart2, (uint8_t*)" ", strlen(" "), HAL_MAX_DELAY);
//        tim7_overflow = 0;
//    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (pulse_fired == 1) {
        pulse_fired = 0;
        if (htim7.Instance->CR1 & TIM_CR1_CEN) {
            // We were measuring a low pulse
            HAL_TIM_Base_Stop_IT(&htim7);

            if((hadc1.Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON) {
//                HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin, GPIO_PIN_RESET);
//                __HAL_ADC_ENABLE(&hadc1);
//                HAL_ADC_Start_IT(&hadc1);
                HAL_ADC_Start_DMA(&hadc1, cur_buf, BUF_SIZE);
            }

//            pulse_time = ((double)tim7_overflow / 16) + ((double)__HAL_TIM_GET_COUNTER(&htim7) / 1000000);
//            sprintf(string_buffer, "\ns: %f ", pulse_time);
//            HAL_UART_Transmit(&huart2, string_buffer, strlen(string_buffer), HAL_MAX_DELAY);
        } else {
            // It's possible on startup/reset/external reset to think we're ready to measure a low pulse,
            // but the signal is high. Prevent that by checking pin state
            if (HAL_GPIO_ReadPin(XY_PULSE_GPIO_Port, XY_PULSE_Pin) == GPIO_PIN_RESET) {
                // Begin measuring a low pulse
                HAL_ADC_Stop_DMA(&hadc1);

                __HAL_TIM_SET_COUNTER(&htim7, 0);
                HAL_TIM_Base_Start_IT(&htim7);
                tim7_overflow = 0;
            }
        }
    }

    if (buf_ready) {
        HAL_GPIO_WritePin(USB_STATUS_PIN_GPIO_Port, USB_STATUS_PIN_Pin, GPIO_PIN_SET);
//        sprintf(string_buffer, "Hello, world!\r\n");
//        CDC_Transmit_FS(string_buffer, strlen(string_buffer));
        uint8_t status;
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
        if (cur_buf == buffer0) {
            status = CDC_Transmit_FS(buffer1, BUF_SIZE);
        } else {
            status = CDC_Transmit_FS(buffer0, BUF_SIZE);
        }
        if (status != USBD_OK) {
            HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin, GPIO_PIN_SET);
        }
        buf_ready = 0;

        HAL_GPIO_WritePin(USB_STATUS_PIN_GPIO_Port, USB_STATUS_PIN_Pin, GPIO_PIN_RESET);
    }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 62499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DEBUG_PIN_Pin|USB_STATUS_PIN_Pin|ADC_STATUS_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HW_SWITCH_Pin */
  GPIO_InitStruct.Pin = HW_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HW_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XY_PULSE_Pin */
  GPIO_InitStruct.Pin = XY_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(XY_PULSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_PIN_Pin USB_STATUS_PIN_Pin ADC_STATUS_PIN_Pin */
  GPIO_InitStruct.Pin = DEBUG_PIN_Pin|USB_STATUS_PIN_Pin|ADC_STATUS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

void SystemClock_SwitchToPLL() {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

/*
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCL_DIV2;
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }

//    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
//    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
//    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    HAL_GPIO_WritePin(ADC_STATUS_PIN_GPIO_Port, ADC_STATUS_PIN_Pin, GPIO_PIN_SET);
    HAL_ADC_Stop_DMA(&hadc1);
    buf_ready = 1;
    if (cur_buf == buffer0) {
        cur_buf = buffer1;
    } else {
        cur_buf = buffer0;
    }
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)cur_buf, BUF_SIZE);
    HAL_GPIO_WritePin(ADC_STATUS_PIN_GPIO_Port, ADC_STATUS_PIN_Pin, GPIO_PIN_RESET);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    // possible overrun
    HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin, GPIO_PIN_SET);
    HAL_DelayUS(100);
    HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin, GPIO_PIN_RESET);
}


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
  while (1) {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
