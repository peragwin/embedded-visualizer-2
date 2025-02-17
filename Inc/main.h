/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
void HandleError(HAL_StatusTypeDef err, int busyOK);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define ROT1_DT_Pin GPIO_PIN_0
#define ROT1_DT_GPIO_Port GPIOF
#define ROT1_CK_Pin GPIO_PIN_2
#define ROT1_CK_GPIO_Port GPIOF
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define USB_OTG_FS_OVCR_Pin GPIO_PIN_7
#define USB_OTG_FS_OVCR_GPIO_Port GPIOG
#define LCD_RS_Pin GPIO_PIN_9
#define LCD_RS_GPIO_Port GPIOC
#define LCD_DC_Pin GPIO_PIN_10
#define LCD_DC_GPIO_Port GPIOC
#define ROT0_DT_Pin GPIO_PIN_0
#define ROT0_DT_GPIO_Port GPIOD
#define ROT0_CK_Pin GPIO_PIN_1
#define ROT0_CK_GPIO_Port GPIOD
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define ROT0_IRQn EXTI1_IRQn
#define ROT1_IRQn EXTI2_IRQn
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
