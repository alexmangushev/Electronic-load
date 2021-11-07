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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I_Pin GPIO_PIN_0
#define I_GPIO_Port GPIOA
#define U_Pin GPIO_PIN_1
#define U_GPIO_Port GPIOA
#define SD_detected_Pin GPIO_PIN_3
#define SD_detected_GPIO_Port GPIOA
#define SD_detected_EXTI_IRQn EXTI3_IRQn
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define RST_Pin GPIO_PIN_0
#define RST_GPIO_Port GPIOB
#define DC_Pin GPIO_PIN_1
#define DC_GPIO_Port GPIOB
#define DIN_Pin GPIO_PIN_2
#define DIN_GPIO_Port GPIOB
#define CLK_Pin GPIO_PIN_10
#define CLK_GPIO_Port GPIOB
#define SPI_CC_Pin GPIO_PIN_12
#define SPI_CC_GPIO_Port GPIOB
#define LDAC_Pin GPIO_PIN_8
#define LDAC_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_15
#define FAN_GPIO_Port GPIOA
#define ENC_Button_Pin GPIO_PIN_5
#define ENC_Button_GPIO_Port GPIOB
#define ENC_Button_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
