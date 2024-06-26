/*
 * i2c_slave.h
 *
 *  Created on: Jun 21, 2024
 *      Author: al
 *
 *
 *  Definitionen Send/Recieve
 *  TEMPERATURREGLER 2024
 *
 *  Der Temperaturregler erwartet immer folgende Daten vom Master:
 *  - 1 Byte: was tun (0x01 - nothing, 0xF9-0xFF do something, switch something) aRxBuffer[0]
 *  - 1 Byte: nothing (dann kann man evtl den pointer auf ein uint16_t array dingsen)
 *  - 2 Bytes: Temperatur Einlauf -> wärmer -> mehr stoff geben aRxBuffer[1-2]
 *  - 2 Bytes: Temperatur Kühlkreislauf -> zu hot: runterregeln! aRxBuffer [3-4]
 *  - 2 Bytes: Sonstige Daten aRxBuffer [5-6]
 *
 *  Der Temperaturregler schickt _immer_ folgendes zurück
 *  - 2 Bytes: Temperatur Block Aussen
 *  - 2 Bytes: Temperatur Block Innen
 *  - 1 Byte Batteriestatus (0x01 high, 0x02 normal, 0x03 low, 0x04 ultralow)
 *  - 1 Byte InVoltage *10 (127 = 12,7 V)
 *  - 2 Bytes: Aktueller Stromverbrauch in W
 *  - 2 Bytes: Stromverbrauch in Wh letzte Zapfung (zwischen 2 Zapfende)
 *  - 2 Bytes: Flowrate Kühlwasser
 *
 *  Also:
 *  Master schickt Befehl 0x01, gefolgt von 2 Bytes Einlauf und 2 Bytes Kühlkreislauf
 *  Slave schickt 10 Bytes zurück
 *
 *  Die Daten werden LITTLE ENDIAN übertragen. also aus uint16_t 0xAABB wird:
 *  Übertragung: [0]0xBB [1]0xAA
 *
 *  Daten werden von Funktionen in die globalen Variablen von common.h geschrieben/gelesen
 *
 *  FreeRTOS kuckt dann ob befehlVomMaster > 1 ist und macht was eventuell
 *
 *  Den Rest stellen wir als LiveExpression ein im Debug mode - keine Übertragung notwendig
 *
 *
 *
 *  nur zum merken, nehmen wir nicht mehr her
 *  to write in an array: memcpy(buffer, &value, sizeof(uint16_t)).
 *******************************************************************************/
#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_

#define TEMP_RX_BYTES 8
#define TEMP_TX_BYTES 12

#include "main.h"
#include <stdint.h>
#include <string.h>
#include "common.h"

extern I2C_HandleTypeDef hi2c1;

void i2c_slaveStartListen(I2C_HandleTypeDef *hi2c);

void i2cRxDataConvert(void);
void i2cTxDataConvert(void);

#endif /* INC_I2C_SLAVE_H_ */
