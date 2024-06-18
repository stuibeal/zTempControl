/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max31865.h"
#include "Modbus.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern modbusHandler_t ModbusH1;
extern modbusHandler_t ModbusH2;
extern uint16_t ModbusDATA1[16];
extern uint16_t ModbusDATA2[16];
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define PT100INNEN_CS_Pin GPIO_PIN_4
#define PT100INNEN_CS_GPIO_Port GPIOA
#define PUMPE_KRAFT_Pin GPIO_PIN_0
#define PUMPE_KRAFT_GPIO_Port GPIOB
#define PUMPE_NORMAL_Pin GPIO_PIN_1
#define PUMPE_NORMAL_GPIO_Port GPIOB
#define PT100AUSSEN_CS_Pin GPIO_PIN_2
#define PT100AUSSEN_CS_GPIO_Port GPIOB
#define FLOWSENSOR_Pin GPIO_PIN_14
#define FLOWSENSOR_GPIO_Port GPIOB
#define FLOWSENSOR_EXTI_IRQn EXTI15_10_IRQn
#define LUEFTER_Pin GPIO_PIN_15
#define LUEFTER_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
