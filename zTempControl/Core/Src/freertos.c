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
modbus_t getData[2];
modbusHandler_t ModbusH0;
modbusHandler_t ModbusH1;

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
float pt100InnenTemp, PIDOut_Innen, TempSetpoint_Innen;
float pt100AussenTemp, PIDOut_Aussen, TempSetpoint_Aussen;

// PT100 Temperaturfühler Aussen
Max31865_t pt100Aussen;
uint8_t pt100AussenIsOK;
uint16_t dpsInputVoltage[2];
uint16_t dpsOutputVoltage[2];
uint16_t dpsOutputCurrent[2];
uint16_t actualOutputPower; // W mit einer Kommastelle
// MODBUS
uint16_t ModbusDataIn0[4];
uint16_t ModbusDataIn1[4];
//POWER
uint64_t wattSekunden;
uint8_t sleepMode = 0;
uint8_t oldSleepMode = 0;
//PUMPE
uint8_t kraftPumpeIsOn = 0;
uint8_t normalPumpeIsOn = 0;
//DEBUG
uint8_t debugBatterieStatus = 10; // kann man dann beim debuggen ändern. bei 10 tut sich nix.
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for checkInVoltage */
osThreadId_t checkInVoltageHandle;
const osThreadAttr_t checkInVoltage_attributes = {
  .name = "checkInVoltage",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pidTask */
osThreadId_t pidTaskHandle;
const osThreadAttr_t pidTask_attributes = {
  .name = "pidTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for checkPower */
osThreadId_t checkPowerHandle;
const osThreadAttr_t checkPower_attributes = {
  .name = "checkPower",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for checkTemp */
osThreadId_t checkTempHandle;
const osThreadAttr_t checkTemp_attributes = {
  .name = "checkTemp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for dpsTalk */
osThreadId_t dpsTalkHandle;
const osThreadAttr_t dpsTalk_attributes = {
  .name = "dpsTalk",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for sleepControlTas */
osThreadId_t sleepControlTasHandle;
const osThreadAttr_t sleepControlTas_attributes = {
  .name = "sleepControlTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for pumpenTask */
osThreadId_t pumpenTaskHandle;
const osThreadAttr_t pumpenTask_attributes = {
  .name = "pumpenTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for controlTempSetp */
osThreadId_t controlTempSetpHandle;
const osThreadAttr_t controlTempSetp_attributes = {
  .name = "controlTempSetp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for flowRateTimer */
osTimerId_t flowRateTimerHandle;
const osTimerAttr_t flowRateTimer_attributes = {
  .name = "flowRateTimer"
};
/* Definitions for wattSekundenTimer */
osTimerId_t wattSekundenTimerHandle;
const osTimerAttr_t wattSekundenTimer_attributes = {
  .name = "wattSekundenTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
void kraftPumpe(uint8_t offon);
void normalPumpe(uint8_t offon);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartCheckInV(void *argument);
void StartPidTask(void *argument);
void StartCheckPower(void *argument);
void startCheckTemp(void *argument);
void StartDpsTalk(void *argument);
void StartSleepControl(void *argument);
void StartPumpenTask(void *argument);
void StartControlTempSet(void *argument);
void flowRateCallback(void *argument);
void wattSekundenCallback(void *argument);

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
  flowRateTimerHandle = osTimerNew(flowRateCallback, osTimerPeriodic, NULL, &flowRateTimer_attributes);

  /* creation of wattSekundenTimer */
  wattSekundenTimerHandle = osTimerNew(wattSekundenCallback, osTimerPeriodic, NULL, &wattSekundenTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of checkInVoltage */
  checkInVoltageHandle = osThreadNew(StartCheckInV, NULL, &checkInVoltage_attributes);

  /* creation of pidTask */
  pidTaskHandle = osThreadNew(StartPidTask, NULL, &pidTask_attributes);

  /* creation of checkPower */
  checkPowerHandle = osThreadNew(StartCheckPower, NULL, &checkPower_attributes);

  /* creation of checkTemp */
  checkTempHandle = osThreadNew(startCheckTemp, NULL, &checkTemp_attributes);

  /* creation of dpsTalk */
  dpsTalkHandle = osThreadNew(StartDpsTalk, NULL, &dpsTalk_attributes);

  /* creation of sleepControlTas */
  sleepControlTasHandle = osThreadNew(StartSleepControl, NULL, &sleepControlTas_attributes);

  /* creation of pumpenTask */
  pumpenTaskHandle = osThreadNew(StartPumpenTask, NULL, &pumpenTask_attributes);

  /* creation of controlTempSetp */
  controlTempSetpHandle = osThreadNew(StartControlTempSet, NULL, &controlTempSetp_attributes);

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
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */

	osTimerStart(flowRateTimerHandle, 10000); //startet den FlowRate Timer
	osTimerStart(wattSekundenTimerHandle, 500); //alle 0,5s die Wattzahl mitschreiben
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
void StartCheckInV(void *argument)
{
  /* USER CODE BEGIN StartCheckInV */
	/* Infinite loop */
	uint16_t eingangsSpannungen[16] = { 1260, 1260, 1260, 1260, 1260, 1260,
			1260, 1260, 1260, 1260, 1260, 1260, 1260, 1260, 1260, 1260 };
	uint32_t komplettspannung = 0;
	uint8_t zaehler = 0;

	/* Infinite loop */
	for (;;) {
		//Alle 5000ms die Spannung checken, aber nur wenn nicht gezapft wird
		if (!zapfBool) {
			if (dpsInputVoltage[0] > 0) {
				eingangsSpannungen[zaehler] = dpsInputVoltage[0];
			}
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

		if (debugBatterieStatus < 10) {
			batterieStatus = debugBatterieStatus;
		}
		dpsSetVoltage(dpsInputVoltage[0] - 115); //muss 1V unter U_in sein
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
void StartPidTask(void *argument)
{
  /* USER CODE BEGIN StartPidTask */
	TempSetpoint_Aussen = AUSSEN_SET_TEMP;
	TempSetpoint_Innen = INNEN_SET_TEMP;

	PID(&TPID_Aussen, &pt100AussenTemp, &PIDOut_Aussen, &TempSetpoint_Aussen,
	AUSSEN_KP,
	AUSSEN_KI, AUSSEN_KD, _PID_P_ON_E, _PID_CD_REVERSE);
	PID(&TPID_Innen, &pt100InnenTemp, &PIDOut_Innen, &TempSetpoint_Innen,
	INNEN_KP,
	INNEN_KI, INNEN_KD, _PID_P_ON_E, _PID_CD_REVERSE);

	PID_SetMode(&TPID_Aussen, _PID_MODE_AUTOMATIC);
	PID_SetMode(&TPID_Innen, _PID_MODE_AUTOMATIC);

	PID_SetSampleTime(&TPID_Aussen, 1); //macht freeRTOS!
	PID_SetSampleTime(&TPID_Innen, 1); //macht freeRTOS!

	PID_SetOutputLimits(&TPID_Aussen, AUSSEN_MIN_A, AUSSEN_MAX_A);
	PID_SetOutputLimits(&TPID_Innen, INNEN_MIN_A, INNEN_MAX_A);

	/* Infinite loop */
	for (;;) {
		PID_Compute(&TPID_Aussen);
		PID_Compute(&TPID_Innen);
		osDelay(250);

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
void StartCheckPower(void *argument)
{
  /* USER CODE BEGIN StartCheckPower */
	/*
	 * je nach Eingangsspannung die maximale Outputpower festlegen
	 * alle 10s reicht dick
	 */
	/* Infinite loop */
	for (;;) {
		switch (batterieStatus) {
		case BATT_ULTRAHIGH:
			dpsOnOff(1);
			dpsSetBacklight(5);
			PID_SetOutputLimits(&TPID_Aussen, AUSSEN_MIN_A, 1800);
			PID_SetOutputLimits(&TPID_Innen, INNEN_MIN_A, 1800);
			break;
		case BATT_HIGH:
			dpsOnOff(1);
			dpsSetBacklight(5);
			PID_SetOutputLimits(&TPID_Aussen, AUSSEN_MIN_A, 1500);
			PID_SetOutputLimits(&TPID_Innen, INNEN_MIN_A, 1200);
			break;
		case BATT_NORMAL:
			dpsOnOff(1);
			dpsSetBacklight(5);
			PID_SetOutputLimits(&TPID_Aussen, AUSSEN_MIN_A, 1000);
			PID_SetOutputLimits(&TPID_Innen, INNEN_MIN_A, 500);
			break;
		case BATT_LOW:
			dpsOnOff(1);
			dpsSetBacklight(1);
			PID_SetOutputLimits(&TPID_Aussen, AUSSEN_MIN_A, 200);
			PID_SetOutputLimits(&TPID_Innen, INNEN_MIN_A, 200);
			break;
		case BATT_ULTRALOW:
			dpsSetBacklight(0);
			dpsOnOff(0);
			break;
		}
		osDelay(10000);

	}
  /* USER CODE END StartCheckPower */
}

/* USER CODE BEGIN Header_startCheckTemp */
/**
 *
 * @brief Function implementing the checkTemp thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startCheckTemp */
void startCheckTemp(void *argument)
{
  /* USER CODE BEGIN startCheckTemp */
	/* Infinite loop */
	for (;;) {
		float t;
		pt100InnenIsOK = Max31865_readTempC(&pt100Innen, &t);
		pt100InnenTemp = Max31865_Filter(t, pt100InnenTemp, 0.08); //  << For Smoothing data
		blockInnenTemp = (int16_t) (pt100InnenTemp * 100);
		osDelay(490);
		pt100AussenIsOK = Max31865_readTempC(&pt100Aussen, &t);
		pt100AussenTemp = Max31865_Filter(t, pt100AussenTemp, 0.08); //  << For Smoothing data
		blockAussenTemp = (int16_t) (pt100AussenTemp * 100);
		osDelay(490);

	}
  /* USER CODE END startCheckTemp */
}

/* USER CODE BEGIN Header_StartDpsTalk */
/**
 *
 * @brief Function implementing the dpsTalk thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDpsTalk */
void StartDpsTalk(void *argument)
{
  /* USER CODE BEGIN StartDpsTalk */
	dpsSetCurrent(INNEN_MIN_A);
	osDelay(150);
	uint32_t u32NotificationValue;
	getData[0].u8id = 1; // slave address
	getData[0].u8fct = MB_FC_READ_REGISTERS; // function code
	getData[0].u16RegAdd = 0x02; // start address in slave
	getData[0].u16CoilsNo = 4; // number of elements (coils or registers) to read
	getData[0].u16reg = ModbusDataIn0; // pointer to a memory array in the Arduino

	getData[1].u8id = 1; // slave address
	getData[1].u8fct = MB_FC_READ_REGISTERS; // function code
	getData[1].u16RegAdd = 0x02; // start address in slave
	getData[1].u16CoilsNo = 4; // number of elements (coils or registers) to read
	getData[1].u16reg = ModbusDataIn1; // pointer to a memory array

	/* Infinite loop */
	for (;;) {
		if (zapfBool) {
			dpsSetCurrent((uint16_t) PIDOut_Aussen);
		} else {
			dpsSetCurrent((uint16_t) PIDOut_Innen);
		}

		ModbusQuery(&ModbusH0, getData[0]); // make a query
		u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
		if (u32NotificationValue != ERR_OK_QUERY) {
			dpsError();
		}
		dpsOutputVoltage[0] = ModbusDataIn0[0];
		dpsOutputCurrent[0] = ModbusDataIn0[1];
		dpsInputVoltage[0] = ModbusDataIn0[3];

		osDelay(100);

		ModbusQuery(&ModbusH1, getData[1]); // make a query
		u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
		if (u32NotificationValue != ERR_OK_QUERY) {
			dpsError();
		}
		dpsOutputVoltage[1] = ModbusDataIn1[0];
		dpsOutputCurrent[1] = ModbusDataIn1[1];
		dpsInputVoltage[1] = ModbusDataIn1[3];

		osDelay(100);

		eingangsSpannung = (dpsInputVoltage[0] + dpsInputVoltage[1]) / 20; //schnittmenge mit 1 kommastelle

	}
  /* USER CODE END StartDpsTalk */
}

/* USER CODE BEGIN Header_StartSleepControl */
/**
 * @brief Function implementing the sleepControlTas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSleepControl */
void StartSleepControl(void *argument)
{
  /* USER CODE BEGIN StartSleepControl */
	/* Infinite loop */
	for (;;) {
		if (sleepMode && !oldSleepMode) {
			oldSleepMode = sleepMode;
			osTimerStop(wattSekundenTimerHandle);
			osTimerStop(flowRateTimerHandle);
			vTaskSuspend(defaultTaskHandle);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
			vTaskSuspend(checkInVoltageHandle);
			vTaskSuspend(checkPowerHandle);
			vTaskSuspend(checkTempHandle);
			vTaskSuspend(pidTaskHandle);
			vTaskSuspend(dpsTalkHandle);
			vTaskSuspend(pumpenTaskHandle);
			vTaskSuspend(controlTempSetpHandle);
			dpsOnOff(0);
			osDelay(100);
			dpsSetLock(1);
			osDelay(100);
			dpsSetBacklight(5);
			osDelay(100);
			dpsSetVoltage(0);
			osDelay(100);
			dpsSetCurrent(0);
			osDelay(500);
			kraftPumpe(0);
			dpsSetBacklight(4);
			osDelay(1000);
			normalPumpe(0);
			dpsSetBacklight(3);
			osDelay(1000);
			HAL_GPIO_WritePin(LUEFTERS_GPIO_Port, LUEFTERS_Pin, 0);
			dpsSetBacklight(2);
			osDelay(1000);
			dpsSetBacklight(1);
			osDelay(1000);
			dpsSetBacklight(0);
			osDelay(1000);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		}

		if (!sleepMode && oldSleepMode) {
			oldSleepMode = sleepMode;
			osTimerStart(wattSekundenTimerHandle, 500);
			osTimerStart(flowRateTimerHandle, 10000);
			vTaskResume(defaultTaskHandle);
			vTaskResume(checkInVoltageHandle);
			vTaskResume(checkPowerHandle);
			vTaskResume(checkTempHandle);
			vTaskResume(pidTaskHandle);
			vTaskResume(dpsTalkHandle);
			vTaskResume(pumpenTaskHandle);
			vTaskResume(controlTempSetpHandle);
			dpsSetLock(1);
			kraftPumpe(1);
			osDelay(5000);
			normalPumpe(1);
			osDelay(1000);
			dpsSetBacklight(5);
			osDelay(1000);
			dpsSetCurrent(INNEN_MIN_A);
			osDelay(1000);
			dpsOnOff(1);
		}
		osDelay(500);
	}
  /* USER CODE END StartSleepControl */
}

/* USER CODE BEGIN Header_StartPumpenTask */
/**
 * @brief Function implementing the pumpenTask thread.
 * 		 Hier wird gecheckt ob die große Pumpe gebraucht wird
 *
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPumpenTask */
void StartPumpenTask(void *argument)
{
  /* USER CODE BEGIN StartPumpenTask */
	kraftPumpe(1);
	osDelay(5000);
	normalPumpe(1);
	/* Infinite loop */
	osDelay(5000); // GIVE HIM TIME!!!!!!!!!!1!11!!
	for (;;) {
		osDelay(1000);
		if (batterieStatus < BATT_ULTRALOW) {
			if (stromVerbrauchAktuell > 9900 && !kraftPumpeIsOn) {
				kraftPumpe(1);
			}
			if (kuehlWasserTemp > 3500 && !kraftPumpeIsOn){
				kraftPumpe(1);
			}

			if (stromVerbrauchAktuell < 1 && kuehlWasserTemp < 3500) {
				kraftPumpe(0);
				normalPumpe(0);
			}

			if (stromVerbrauchAktuell > 0
					&& (!kraftPumpeIsOn && !normalPumpeIsOn)) {
				kraftPumpe(1);
				osDelay(5000); //delay weil die normale Pumpe das am Anfang nicht schafft
				normalPumpe(1);
			}

			if (stromVerbrauchAktuell < 9900 && kraftPumpeIsOn) {
				normalPumpe(1);
				osDelay(1000);
				kraftPumpe(0);
			}
		} else {
			/* no power, no PUmpe */
			kraftPumpe(0);
			osDelay(2000);
			normalPumpe(0);
		}
	}
	osDelay(567);

  /* USER CODE END StartPumpenTask */
}

/* USER CODE BEGIN Header_StartControlTempSet */
/**
 * @brief Function implementing the controlTempSetp thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlTempSet */
void StartControlTempSet(void *argument)
{
  /* USER CODE BEGIN StartControlTempSet */
	TempSetpoint_Aussen = lastUserSetTemp;
	TempSetpoint_Innen = lastUserSetTemp;
	/* Infinite loop */
	for (;;) {
		if (kuehlWasserTemp-2800 > (uint16_t)(lastUserSetTemp *100)){
			lastUserSetTemp = (float)((kuehlWasserTemp/100-28));
		}

		switch (batterieStatus) {
		case BATT_ULTRAHIGH:
			/* kühl weg den Strom! Sonst geht die Batterie über!*/
			if (kuehlWasserTemp < 3500) {
				TempSetpoint_Aussen = 2.5f;
				TempSetpoint_Innen = 1.5f;
			} else
			{
				TempSetpoint_Aussen = lastUserSetTemp;
				TempSetpoint_Innen = lastUserSetTemp;
			}


			break;
		case BATT_HIGH:
			if (lastUserSetTemp > 3) {
				TempSetpoint_Aussen = lastUserSetTemp;
				TempSetpoint_Innen = lastUserSetTemp;
			} else {
				TempSetpoint_Aussen = 3.0f;
				TempSetpoint_Innen = 4.0f;
			}
			break;
		case BATT_NORMAL:
			if (lastUserSetTemp > 4) {
				TempSetpoint_Aussen = lastUserSetTemp;
				TempSetpoint_Innen = lastUserSetTemp;
			} else {
				TempSetpoint_Aussen = 4.0f;
				TempSetpoint_Innen = 4.0f;
			}
			break;
		case BATT_LOW:
			/* BEDA! NOCHSCHIERN! */
			TempSetpoint_Aussen = 14.0f;
			TempSetpoint_Innen = 12.0f;
			break;
		case BATT_ULTRALOW:
			/* drink warm beer you fool!*/
			TempSetpoint_Aussen = 20.0f;
			TempSetpoint_Innen = 20.0f;
			break;
		}
		osDelay(200);
	}
  /* USER CODE END StartControlTempSet */
}

/* flowRateCallback function */
void flowRateCallback(void *argument)
{
  /* USER CODE BEGIN flowRateCallback */
	/* nimmt alle 10s die vom Flowmeter gemessenen Daten und teilt sie durch zehn sekunden :) */
	kuehlWasserFlowRate = (uint16_t) (flowCounter / 50); //wir nehmens einfach mal durch 50....
	flowCounter = 0;
	if (kuehlWasserFlowRate == 0 && stromVerbrauchAktuell > 0) {
		HAL_GPIO_WritePin(LUEFTERS_GPIO_Port, LUEFTERS_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LUEFTERS_GPIO_Port, LUEFTERS_Pin, 0);
	}
	if (kuehlWasserTemp > 3500) {
		HAL_GPIO_WritePin(LUEFTERS_GPIO_Port, LUEFTERS_Pin, 1);
	}
  /* USER CODE END flowRateCallback */
}

/* wattSekundenCallback function */
void wattSekundenCallback(void *argument)
{
  /* USER CODE BEGIN wattSekundenCallback */
	actualOutputPower = (dpsOutputCurrent[0] / 10) * (dpsOutputVoltage[0] / 10);
	actualOutputPower += (dpsOutputCurrent[1] / 10)
			* (dpsOutputVoltage[1] / 10);

	stromVerbrauchAktuell = actualOutputPower;  //jetzt W mit 2 Nackommastellen!
	wattSekunden += stromVerbrauchAktuell / 2;
	stromVerbrauchLetzteZapfung = (uint16_t) (wattSekunden / 36000);
  /* USER CODE END wattSekundenCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void kraftPumpe(uint8_t onoff) {

	if (onoff) {
		HAL_GPIO_WritePin(PUMPE_KRAFT_GPIO_Port, PUMPE_KRAFT_Pin, 1);
		kraftPumpeIsOn = 1;
	} else {
		HAL_GPIO_WritePin(PUMPE_KRAFT_GPIO_Port, PUMPE_KRAFT_Pin, 0);
		kraftPumpeIsOn = 0;
	}
}

void normalPumpe(uint8_t onoff) {
	if (onoff) {
		HAL_GPIO_WritePin(PUMPE_NORMAL_GPIO_Port, PUMPE_NORMAL_Pin, 1);
		normalPumpeIsOn = 1;
	} else {
		HAL_GPIO_WritePin(PUMPE_NORMAL_GPIO_Port, PUMPE_NORMAL_Pin, 0);
		normalPumpeIsOn = 0;
	}

}
/* USER CODE END Application */

