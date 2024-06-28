/*
 * dpsControl.c
 *
 *  Created on: Jun 27, 2024
 *      Author: stuibeal
 */
#include "dpsControl.h"
#include "main.h"
#include "common.h"
#include "stdint.h"
#include "string.h"
#include "Modbus.h"
#include "usart.h"

modbus_t telegram[2];
modbusHandler_t ModbusH0;
uint16_t ModbusDATA0[16];
modbusHandler_t ModbusH1;
uint16_t ModbusDATA1[16];

void startDPS(void) {
	/* MODBUS RTU Master 1 initialization */
	ModbusH0.uModbusType = MB_MASTER;
	ModbusH0.port = &huart1;
	ModbusH0.u8id = 0; // For master it must be 0
	ModbusH0.u16timeOut = 1000;
	ModbusH0.EN_Port = NULL;
	ModbusH0.EN_Port = NULL;
	ModbusH0.EN_Pin = 0;
	ModbusH0.u16regs = ModbusDATA0;
	ModbusH0.u16regsize = sizeof(ModbusDATA0) / sizeof(ModbusDATA0[0]);
	ModbusH0.xTypeHW = USART_HW;
	//Initialize Modbus library
	ModbusInit(&ModbusH0);
	//Start capturing traffic on serial Port
	ModbusStart(&ModbusH0);

	/* MODBUS RTU Master 2 initialization */
	ModbusH1.uModbusType = MB_MASTER;
	ModbusH1.port = &huart2;
	ModbusH1.u8id = 0; // For master it must be 0
	ModbusH1.u16timeOut = 1000;
	ModbusH1.EN_Port = NULL;
	ModbusH1.EN_Port = NULL;
	ModbusH1.EN_Pin = 0;
	ModbusH1.u16regs = ModbusDATA1;
	ModbusH1.u16regsize = sizeof(ModbusDATA1) / sizeof(ModbusDATA1[0]);
	ModbusH1.xTypeHW = USART_HW;
	//Initialize Modbus library
	ModbusInit(&ModbusH1);
	//Start capturing traffic on serial Port
	ModbusStart(&ModbusH1);
}

uint16_t getEingangsSpannung(void) {
	return dps0getSingleRegister(RD_UIN);
}

uint16_t getWatt(void) {
	uint16_t watt = 0;
	watt += dps0getSingleRegister(RD_POWER);
	watt += dps1getSingleRegister(RD_POWER);
	return watt;
}

/**
 * @brief Sperrt das Userinterface der beiden DPS
 */
void dpsSetLock(uint16_t yesno) {
	dpsSetSingleRegister(RD_KEYLOCK, yesno);
}

void dpsSetBacklight(uint16_t backlightvalue) {
	dpsSetSingleRegister(RD_BACKLIGHT, backlightvalue);
}

void dpsOnOff(uint16_t onoff) {
	dpsSetSingleRegister(RD_ONOFF, onoff);

}
uint16_t getAusgangsSpannung(void) {
	uint16_t spannung = 0;
	spannung += dps0getSingleRegister(RD_UOUT);
	spannung += dps1getSingleRegister(RD_UOUT);
	return (spannung/2);
}
uint16_t getAusgangsStrom(void) {
	uint16_t strom = 0;
	strom += dps0getSingleRegister(RD_IOUT);
	strom += dps1getSingleRegister(RD_IOUT);
	return (strom/2);

}
void dpsSetCurrent(uint16_t ampere) {
	dpsSetSingleRegister(RD_I_SET, ampere);

}
void dpsSetVoltage(uint16_t volt) {
	dpsSetSingleRegister(RD_U_SET, volt);
}
void dpsSetSingleRegister(uint8_t dpsRegister, uint16_t dpsData) {
	uint32_t u32NotificationValue;
	ModbusDATA0[0] = dpsData;
	telegram[0].u8id = 1; // slave address
	telegram[0].u8fct = MB_FC_WRITE_REGISTER; // function code
	telegram[0].u16RegAdd = dpsRegister; // start address in slave
	telegram[0].u16CoilsNo = 1; // number of elements (coils or registers) to read
	telegram[0].u16reg = ModbusDATA0; // pointer to a memory array in the Arduino

	ModbusQuery(&ModbusH0, telegram[0]); // make a query
	u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
	if (u32NotificationValue != ERR_OK_QUERY) {
		dpsError();
	}

	ModbusDATA1[0] = dpsData;
	telegram[1].u8id = 1; // slave address
	telegram[1].u8fct = MB_FC_WRITE_REGISTER; // function code
	telegram[1].u16RegAdd = dpsRegister; // start address in slave
	telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
	telegram[1].u16reg = ModbusDATA1; // pointer to a memory array in the Arduino

	ModbusQuery(&ModbusH1, telegram[1]); // make a query
	u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
	if (u32NotificationValue != ERR_OK_QUERY) {
		dpsError();
	}

}

uint16_t dps0getSingleRegister(uint8_t dpsRegister) {
	uint32_t u32NotificationValue;
	telegram[0].u8id = 1; // slave address
	telegram[0].u8fct = MB_FC_READ_REGISTERS; // function code
	telegram[0].u16RegAdd = dpsRegister; // start address in slave
	telegram[0].u16CoilsNo = 2; // number of elements (coils or registers) to read
	telegram[0].u16reg = ModbusDATA0; // pointer to a memory array in the Arduino

	ModbusQuery(&ModbusH0, telegram[0]); // make a query
	u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
	if (u32NotificationValue != ERR_OK_QUERY) {
		dpsError();
	}
	return ModbusDATA0[0];
}

uint16_t dps1getSingleRegister(uint8_t dpsRegister) {
	uint32_t u32NotificationValue;
	telegram[1].u8id = 1; // slave address
	telegram[1].u8fct = MB_FC_READ_REGISTERS; // function code
	telegram[1].u16RegAdd = dpsRegister; // start address in slave
	telegram[1].u16CoilsNo = 2; // number of elements (coils or registers) to read
	telegram[1].u16reg = ModbusDATA1; // pointer to a memory array in the Arduino

	ModbusQuery(&ModbusH1, telegram[1]); // make a query
	u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
	if (u32NotificationValue != ERR_OK_QUERY) {
		dpsError();
	}
	return ModbusDATA1[1];
}

void dpsError(void) {
	//Error_Handler();
}

