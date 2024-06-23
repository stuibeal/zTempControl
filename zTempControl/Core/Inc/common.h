/*
 * common.h
 *
 *  Created on: Jun 22, 2024
 *      Author: al
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_
//Leider brauchen wir ein paar globale Variablen
//Modbus
#include "Modbus.h"

extern modbusHandler_t ModbusH1;
extern uint16_t ModbusDATA1[16];
extern modbusHandler_t ModbusH2;
extern uint16_t ModbusDATA2[16];

//I2C und Register
extern uint16_t dataRxRegister[4];
extern uint16_t dataTxRegister[6];



#endif /* INC_COMMON_H_ */
