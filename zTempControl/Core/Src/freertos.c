/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi.h"
#include "gpio.h"
#include <stdint.h>
#include "max31865.h"
#include "usb_device.h"
#include <stdio.h>
#include "semphr.h"
#include "Modbus.h"
#include "common.h"

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
/* USER CODE BEGIN Variables */
// PT100 Temperaturfühler Innen
Max31865_t pt100Innen;
uint8_t pt100InnenIsOK;
float pt100InnenTemp;
// PT100 Temperaturfühler Aussen
Max31865_t pt100Aussen;
uint8_t pt100AussenIsOK;
float pt100AussenTemp;
// MODBUS
modbus_t telegram[2];
// DPS5020

char buffer[128];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for checkTemp */
osThreadId_t checkTempHandle;
const osThreadAttr_t checkTemp_attributes = { .name = "checkTemp", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for dspTalkTask */
osThreadId_t dspTalkTaskHandle;
const osThreadAttr_t dspTalkTask_attributes = { .name = "dspTalkTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void checkTempStart(void *argument);
void startDpsTalk(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */
	Max31865_init(&pt100Innen, &hspi1, PT100INNEN_CS_GPIO_Port,
	PT100INNEN_CS_Pin, 3, 50);
	Max31865_init(&pt100Aussen, &hspi1, PT100AUSSEN_CS_GPIO_Port,
	PT100AUSSEN_CS_Pin, 3, 50);

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of checkTemp */
	checkTempHandle = osThreadNew(checkTempStart, NULL, &checkTemp_attributes);

	/* creation of dspTalkTask */
	dspTalkTaskHandle = osThreadNew(startDpsTalk, NULL,
			&dspTalkTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		//Hier nur ein Lebenszeichen von sich geben
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		osDelay(1000);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_checkTempStart */
/**
 * @brief Function implementing the checkTemp thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_checkTempStart */
void checkTempStart(void *argument) {
	/* USER CODE BEGIN checkTempStart */
	/* Infinite loop */
	for (;;) {
		float t;
		pt100InnenIsOK = Max31865_readTempC(&pt100Innen, &t);
		pt100InnenTemp = Max31865_Filter(t, pt100InnenTemp, 0.08); //  << For Smoothing data
		blockInnenTemp = (int16_t) (pt100InnenTemp * 100);

		osDelay(500);
		pt100AussenIsOK = Max31865_readTempC(&pt100Aussen, &t);
		pt100AussenTemp = Max31865_Filter(t, pt100AussenTemp, 0.08); //  << For Smoothing data
		blockAussenTemp = (int16_t) (pt100AussenTemp * 100);

		osDelay(500);
	}
	/* USER CODE END checkTempStart */
}

/* USER CODE BEGIN Header_startDpsTalk */
/**
 * @brief Function implementing the dspTalkTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startDpsTalk */
void startDpsTalk(void *argument) {
	/* USER CODE BEGIN startDpsTalk */
	modbus_t telegram;
	uint32_t u32NotificationValue;
	uint16_t eingangsSpannungen[16] = { 1260, 1260, 1260, 1260, 1260, 1260, 1260, 1260,
			1260, 1260, 1260, 1260, 1260, 1260, 1260, 1260};
	uint32_t komplettspannung = 0;
	uint8_t zaehler = 0;

	telegram.u8id = 1; // slave address
	telegram.u8fct = MB_FC_READ_REGISTERS; // function code
	telegram.u16RegAdd = RD_UIN; // start address in slave
	telegram.u16CoilsNo = 2; // number of elements (coils or registers) to read
	telegram.u16reg = ModbusDATA2; // pointer to a memory array in the Arduino
	/* Infinite loop */
	for (;;) {
		//Alle 500ms die Spannung checken, aber nur wenn nicht gezapft wird
		if (!zapfBool) {
			ModbusQuery(&ModbusH2, telegram); // make a query
			u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
			if (u32NotificationValue != ERR_OK_QUERY) {
				//handle error
				//  while(1);
			}
			//zum glätten der Eingangsspannung

			eingangsSpannungen[zaehler] = ModbusDATA2[0];
			zaehler++;
			if (zaehler >15) {
				zaehler = 0;
			}

			komplettspannung = 0;
			for (uint8_t i = 0; i<16; i++) {
				komplettspannung += eingangsSpannungen[i];
			}
			eingangsSpannung = (uint8_t) (komplettspannung / 160);

			//check hier mal den batteriestatus
			if (eingangsSpannung < 110) {
				batterieStatus = BATT_ULTRALOW;
			}
			if (eingangsSpannung >= 110) {
				batterieStatus = BATT_LOW;
			}
			if (eingangsSpannung > 120) {
				batterieStatus = BATT_NORMAL;
			}
			if (eingangsSpannung > 130) {
				batterieStatus = BATT_HIGH;
			}
			if (eingangsSpannung > 140) {
				batterieStatus = BATT_ULTRAHIGH;
			}

		}
		osDelay(5000);
	}
	/* USER CODE END startDpsTalk */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

