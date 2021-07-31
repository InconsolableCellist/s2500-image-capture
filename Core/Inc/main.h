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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t switch_pressed;
extern uint8_t tim7_overflow;
extern uint8_t pulse_fired;
extern uint8_t adc_int;
extern uint8_t usb_transfer_complete;
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
#define HW_SWITCH_Pin GPIO_PIN_0
#define HW_SWITCH_GPIO_Port GPIOA
#define HW_SWITCH_EXTI_IRQn EXTI0_IRQn
#define XY_PULSE_Pin GPIO_PIN_1
#define XY_PULSE_GPIO_Port GPIOA
#define XY_PULSE_EXTI_IRQn EXTI1_IRQn
#define DEBUG_PIN_Pin GPIO_PIN_4
#define DEBUG_PIN_GPIO_Port GPIOA
#define USB_STATUS_PIN_Pin GPIO_PIN_6
#define USB_STATUS_PIN_GPIO_Port GPIOA
#define ADC_STATUS_PIN_Pin GPIO_PIN_7
#define ADC_STATUS_PIN_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
