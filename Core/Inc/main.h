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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define uwPrescalerValueTIM8 0
#define uwPrescalerValueTIM1 0
#define PeriodTIM3 0
#define ADC_VDC__Pin GPIO_PIN_0
#define ADC_VDC__GPIO_Port GPIOC
#define ADC_VDC_C1_Pin GPIO_PIN_1
#define ADC_VDC_C1_GPIO_Port GPIOC
#define ADC_IA_Pin GPIO_PIN_0
#define ADC_IA_GPIO_Port GPIOA
#define ADC_IB_Pin GPIO_PIN_1
#define ADC_IB_GPIO_Port GPIOA
#define ADC_VA_Pin GPIO_PIN_2
#define ADC_VA_GPIO_Port GPIOA
#define ADC_VB_Pin GPIO_PIN_3
#define ADC_VB_GPIO_Port GPIOA
#define LED_HL2_Pin GPIO_PIN_4
#define LED_HL2_GPIO_Port GPIOA
#define LED_HL1_Pin GPIO_PIN_6
#define LED_HL1_GPIO_Port GPIOA
#define Relay_Pin GPIO_PIN_7
#define Relay_GPIO_Port GPIOA
#define PFC_SW_SRC_Pin GPIO_PIN_5
#define PFC_SW_SRC_GPIO_Port GPIOC
#define OCP_A_Pin GPIO_PIN_10
#define OCP_A_GPIO_Port GPIOB
#define TC_HS_Pin GPIO_PIN_12
#define TC_HS_GPIO_Port GPIOB
#define ADC_VC_Pin GPIO_PIN_14
#define ADC_VC_GPIO_Port GPIOB
#define TA_HS_Pin GPIO_PIN_8
#define TA_HS_GPIO_Port GPIOA
#define TB_HS_Pin GPIO_PIN_10
#define TB_HS_GPIO_Port GPIOA
#define AC_FAULT_Pin GPIO_PIN_12
#define AC_FAULT_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
