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
#include "dpsControl.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID_TypeDef TPID_Innen;
PID_TypeDef TPID_Aussen;

float Temp_Innen, PIDOut_Innen, TempSetpoint_Innen;
float Temp_Aussen, PIDOut_Aussen, TempSetpoint_Aussen;



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

char buffer[128];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for checkInVoltage */
osThreadId_t checkInVoltageHandle;
const osThreadAttr_t checkInVoltage_attributes = { .name = "checkInVoltage",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for pidTask */
osThreadId_t pidTaskHandle;
const osThreadAttr_t pidTask_attributes = { .name = "pidTask", .stack_size = 128
		* 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for checkPower */
osThreadId_t checkPowerHandle;
const osThreadAttr_t checkPower_attributes =
		{ .name = "checkPower", .stack_size = 128 * 4, .priority =
				(osPriority_t) osPriorityBelowNormal, };
/* Definitions for checkTemp */
osThreadId_t checkTempHandle;
const osThreadAttr_t checkTemp_attributes = { .name = "checkTemp", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for flowRateTimer */
osTimerId_t flowRateTimerHandle;
const osTimerAttr_t flowRateTimer_attributes = { .name = "flowRateTimer" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartCheckInV(void *argument);
void StartPidTask(void *argument);
void StartCheckPower(void *argument);
void startCheckTemp(void *argument);
void flowRateCallback(void *argument);

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
	startDPS();
	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* Create the timer(s) */
	/* creation of flowRateTimer */
	flowRateTimerHandle = osTimerNew(flowRateCallback, osTimerPeriodic, NULL,
			&flowRateTimer_attributes);

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

	/* creation of checkInVoltage */
	checkInVoltageHandle = osThreadNew(StartCheckInV, NULL,
			&checkInVoltage_attributes);

	/* creation of pidTask */
	pidTaskHandle = osThreadNew(StartPidTask, NULL, &pidTask_attributes);

	/* creation of checkPower */
	checkPowerHandle = osThreadNew(StartCheckPower, NULL,
			&checkPower_attributes);

	/* creation of checkTemp */
	checkTempHandle = osThreadNew(startCheckTemp, NULL, &checkTemp_attributes);

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
	osTimerStart(flowRateTimerHandle, 10000); //startet den FlowRate Timer
	dpsSetVoltage(500);
	dpsSetCurrent(2);
	HAL_GPIO_WritePin(PUMPE_KRAFT_GPIO_Port, PUMPE_KRAFT_Pin, 1); //Startet eine Pumpe
	osDelay(5000);
	HAL_GPIO_WritePin(PUMPE_NORMAL_GPIO_Port, PUMPE_NORMAL_Pin, 1); // Startet die andere Pumpe

	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		//Hier nur ein Lebenszeichen von sich geben
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		osDelay(1000);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCheckInV */
/**
 * @brief Function implementing the checkInVoltage thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCheckInV */
void StartCheckInV(void *argument) {
	/* USER CODE BEGIN StartCheckInV */
	/* Infinite loop */
	uint16_t eingangsSpannungen[16] = { 1260, 1260, 1260, 1260, 1260, 1260,
			1260, 1260, 1260, 1260, 1260, 1260, 1260, 1260, 1260, 1260 };
	uint32_t komplettspannung = 0;
	uint8_t zaehler = 0;

	/* Infinite loop */
	for (;;) {
		//Alle 500ms die Spannung checken, aber nur wenn nicht gezapft wird
		if (!zapfBool) {
			eingangsSpannungen[zaehler] = getEingangsSpannung();
			zaehler++;
			if (zaehler > 15) {
				zaehler = 0;
			}
		}

		//zum glätten der Eingangsspannung
		komplettspannung = 0;
		for (uint8_t i = 0; i < 16; i++) {
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
		osDelay(5000);
	}
	/* USER CODE END StartCheckInV */
}

/* USER CODE BEGIN Header_StartPidTask */
/**
 * @brief Function implementing the pidTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPidTask */
void StartPidTask(void *argument) {
	/* USER CODE BEGIN StartPidTask */
	TempSetpoint_Aussen = 4;

	PID(&TPID_Aussen, &pt100AussenTemp, &PIDOut_Aussen, &TempSetpoint_Aussen, 100,
			5, 1, _PID_P_ON_E, _PID_CD_REVERSE);

	PID_SetMode(&TPID_Aussen, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&TPID_Aussen, 1); //macht freeRTOS!
	PID_SetOutputLimits(&TPID_Aussen, 2, 1000); //2=20mA damits nicht zurückwärmt, 1000 = 10.00A -> das sind 480W mein Freund. 20A hält DPS5020 aus pro Stück

	/* Infinite loop */
	for (;;) {
		PID_Compute(&TPID_Aussen);
		dpsSetCurrent((uint16_t)PIDOut_Aussen);
		osDelay(500); //reicht aus, das Ding is eh träg
	}
	/* USER CODE END StartPidTask */
}

/* USER CODE BEGIN Header_StartCheckPower */
/**
 * @brief Function implementing the checkPower thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCheckPower */
void StartCheckPower(void *argument) {
	/* USER CODE BEGIN StartCheckPower */
	/*
	 * Hier wird der Stromverbrauch ermittelt und gedingst!
	 */
	/* Infinite loop */
	for (;;) {
		osDelay(1000);
	}
	/* USER CODE END StartCheckPower */
}

/* USER CODE BEGIN Header_startCheckTemp */
/**
 * @brief Function implementing the checkTemp thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startCheckTemp */
void startCheckTemp(void *argument) {
	/* USER CODE BEGIN startCheckTemp */
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
	/* USER CODE END startCheckTemp */
}

/* flowRateCallback function */
void flowRateCallback(void *argument) {
	/* USER CODE BEGIN flowRateCallback */
	/* nimmt alle 10s die vom Flowmeter gemessenen Daten und teilt sie durch zehn sekunden :) */
	kuehlWasserFlowRate = (uint16_t) (flowCounter / 50); //wir nehmens einfach mal durch 50....
	flowCounter = 0;
	if (kuehlWasserFlowRate == 0) {
		HAL_GPIO_WritePin(LUEFTERS_GPIO_Port, LUEFTERS_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LUEFTERS_GPIO_Port, LUEFTERS_Pin, 0);
	}

	/* USER CODE END flowRateCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

