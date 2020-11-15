/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
  FOUR_PHASE_INIT,
  THREE_PHASE_INIT,
  TWO_PHASE_INIT,
  ONE_PHASE_INIT,
  BURST_INIT,
  FOUR_PHASE,
  THREE_PHASE,
  TWO_PHASE,
  ONE_PHASE,
  BURST,
  OUTPUT_OFF,
  FAULT
} Mode_TypeDef;
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
#define V_FB_Pin GPIO_PIN_0
#define V_FB_GPIO_Port GPIOA
#define I_FB_Pin GPIO_PIN_1
#define I_FB_GPIO_Port GPIOA
#define P1_IFB_Pin GPIO_PIN_4
#define P1_IFB_GPIO_Port GPIOA
#define P2_IFB_Pin GPIO_PIN_5
#define P2_IFB_GPIO_Port GPIOA
#define P3_IFB_Pin GPIO_PIN_6
#define P3_IFB_GPIO_Port GPIOA
#define P4_IFB_Pin GPIO_PIN_7
#define P4_IFB_GPIO_Port GPIOA
#define IMUX_0_Pin GPIO_PIN_0
#define IMUX_0_GPIO_Port GPIOB
#define IMUX_1_Pin GPIO_PIN_1
#define IMUX_1_GPIO_Port GPIOB
#define IMUX_2_Pin GPIO_PIN_2
#define IMUX_2_GPIO_Port GPIOB
#define V_SET_Pin GPIO_PIN_15
#define V_SET_GPIO_Port GPIOB
#define STAT_0_Pin GPIO_PIN_3
#define STAT_0_GPIO_Port GPIOB
#define STAT_1_Pin GPIO_PIN_4
#define STAT_1_GPIO_Port GPIOB
#define STAT_2_Pin GPIO_PIN_5
#define STAT_2_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_6
#define ENABLE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
