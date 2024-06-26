/*
 * zCommunication.h
 *
 *  Created on: Jun 21, 2024
 *      Author: al
 */

#ifndef INC_ZCOMMUNICATION_H_
#define INC_ZCOMMUNICATION_H_

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : zCommunication.h
 * @brief          : Defines für I2C Kommunication
 *                   Sollte für alle uC verwendbar sein
 *
 *                    Die uC übertragen immer drei Bytes über I2C.
 *                    1. Byte: Befehl (hier als const angelegt)
 *                    2. Byte: normal HIGH Byte vom int
 *                    3. Byte: normal LOW Byte vom int
 *
 ******************************************************************************
 * @attention
 *
 * 2024 Z.Gesellschaft / Zapfapparat 2024
 ******************************************************************************
 */
/* USER CODE END Header */

/**
 * Grundlegend I2C Adressen
 */
#define  FLOW_I2C_ADDR  0x12    // Flowzähl uC (in Cube 1 bit linksshiften!)
#define FLOW_I2C_ANTWORTBYTES 2 // die menge an Antwortbytes
#define TEMP_I2C_ADDR 0x13    // Temperaturregel uC

/*
 *  Definitionen für Bytes
 *  FLOWMETER+TEMPERATUR
 *  										ALLES 2 Bytes -> 1 uint16_t
 *  							MASTER		SLAVE			MASTER
 *								SEND		RETURN			SEND
 *******************************************************************************/

#define EBI_MODE 0xF9    // don't know really know
#define BEGIN_ZAPF  0xFA  // Zapfprogramm beginnen
#define END_ZAPF  0xFB    // Zapfprogramm beenden, gezapfte Milliliter übertragen
#define KURZ_VOR_ZAPFENDE 0xFC // sagt das wir kurz vor Ende sind → Valve schließen -> PID auf konservativ
#define LOW_ENERGY = 0xFD // //LEDS nicht benutzen  byte3: 0: vollgas   1: sparen
#define WACH_AUF = 0xFE // alles wieder hochfahren und für den neuen Tag bereiten
#define ZAPFEN_STREICH 0xFF // alles runterfahren, licht aus, ton aus. Halt die Klappe, ich hab Feierabend.

/*
 *  Definitionen für Bytes
 *  FLOWMETER
 *
 *  Der Flowmeter schickt IMMER die aktuellen zapfMillis zurück.
 *  Diese werden dann vom Master durch setUserMilliLitres oder beginZapf wieder
 *  auf 0 zurückgestellt.
 *
 *  								ALLES 2 Bytes -> 1 uint16_t
 *  							MASTER		SLAVE			MASTER
 *								SEND		RETURN			SEND
 *******************************************************************************/
/*** GAUSELMANN (LED FUN 4) HOW TO:
 *  Befehl: 25 (hex!)
 *  Byte 1: Was für LED:
 *          einfach BINÄR eingeben! z.B. 0b01101100 (wär in HEX 6C)
 *  Byte 2: Helligkeit (00-FF)
 *
 */

#define GET_ML  0x01 // sende die gezapften Milliliter. byte 2&3 egal -> schickt ml an Master zurück
#define SET_USER_ML  0x21 // Master schickt die vom User gewählten ml an den Zapf uC
#define LED_FUN_1  0x22 // progrämmsche warp1, parameter: laufzeit, delay, leds von oben nach unten oder so
#define LED_FUN_2  0x23 // progrämmsche warp2, parameter: laufzeit, delay, leds nach mittig zusammen
#define LED_FUN_3  0x24 // progrämmsche warp3, parameter: laufzeit, delay,
#define LED_FUN_4 = 0x25 // progrämmsche GAUSELMANN, parameter: was für LEDS, helligkeit

/*
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
 *  - 2 Bytes: 1 Byte Batteriestatus (0x01 high, 0x02 normal, 0x03 low, 0x04 ultralow), 1 Byte InVoltage *10
 *  - 2 Bytes: Aktueller Stromverbrauch in W
 *  - 2 Bytes: Stromverbrauch in Wh letzte Zapfung (zwischen 2 Zapfende)
 *  - 2 Bytes: Flowrate Kühlwasser
 *
 *  Also:
 *  Master schickt Befehl 0x01, gefolgt von 2 Bytes Einlauf und 2 Bytes Kühlkreislauf
 *  Slave schickt 10 Bytes zurück
 *  Slave checkt im TxCompleteCallback ob er was zu tun hat (aRxBuffer >0x01),
 *  setzt evtl flag für RTOS
 *
 *  Die Daten werden LITTLE ENDIAN übertragen. also aus uint16_t 0xAABB wird:
 *  Übertragung: [0]0xBB [1]0xAA
 *
 *  Den Rest stellen wir als LiveExpression ein im Debug mode - keine Übertragung notwendig
 *
 *  to write in an array: memcpy(buffer, &value, sizeof(uint16_t)).
 *******************************************************************************/
#define JUST_COMMUNICATE 0x01
#define TEMP_RX_BYTES 8
#define TEMP_TX_BYTES 12
#define TEMP_EINLAUF aRxBuffer[1]
#define TEMP_KUEHLKREISLAUF aRxBuffer[2]
#define __SEND_TEMP_BLOCK_AUSSEN aTxBuffer[0]
#define __SEND_TEMP_BLOCK_INNEN aTxBuffer[1]
#define __SEND_BATTERY_VOLTAGE aTxBuffer[2]

/* braucht es nimmer:
 *
#define GET_BLOCK_AUSSEN_TEMP  0x40
#define GET_BLOCK_INNEN_TEMP  0x41
#define GET_IN_VOLTAGE  0x42

 #define transmitInVoltage       0x43  // Data send:   inVoltage in V*100
 #define transmitKuehlFlow       0x44  // Data send:   Durchfluss Kühlwasser (extern) pro 10000ms

 #define setHighTemperatur       0x60  // Data get: Zieltemperatur Block * 100 (2°C)
 #define setMidTemperatur        0x61  // Data get: Normale Temperatur in °C * 100 (6°C)
 #define setLowTemperatur        0x62  // Data get: Energiespar Temperatur * 100 (9°C)
 #define setMinCurrent           0x63  // Data get: Current in mA / 10 (11 = 0,11 A), Untere Regelgrenze
 #define setLowCurrent           0x64  // Data get: current in mA / 10, Obere Regelgrenze bei wenig Strom
 #define setMidCurrent           0x65  // Data get: Current in mA / 10, Obere Regelgrenze bei normalem Strom
 #define setHighCurrent          0x66  // Data get: Current in mA / 10, Obere Regelgrenze bei gutem Strom

 #define setNormVoltage          0x68  // Data get: norm Voltage * 100, passt normal, mehr als 9V macht wenig Sinn bei den Peltierelementen
 #define setMaxVoltage           0x69  // Data get: max Voltage * 100, das wäre dann eigentlich die Batteriespannung
 #define setLowBatteryVoltage    0x6A  // 11V Eingangsspannung
 #define setMidBatteryVoltage    0x6B  // 12V Eingangsspannung
 #define setHighBatteryVoltage   0x6C  // 13V Eingangsspannung

 #define setWasserTemp           0x6D  // Data get: kühlwasserTemp in °C*100 vom DS18B20 Sensor vom Master: Fühler neben Peltier
 #define setEinlaufTemp          0x6E  // Data get: Biertemperatur in °C*100 vom DS18B20 Sensor vom Master: Bierzulauf

 #define setConsKp               0x70  // Data get: konservativer Kp
 #define setConsKi               0x71  // Data get: konservativer Ki
 #define setConsKd               0x72  // Data get: konservativer Kd
 #define setAggKp                0x73  // Data get: aggressiver Kp
 #define setAggKi                0x74  // Data get: aggressiver Ki
 #define setAggKd                0x75  // Data get: aggressiver Kd
 #define setUnterschiedAggPid    0x75  // mal zehn grad nehmen ab wann der aggressiv regelt
 #define setSteuerZeit           0x76  // alle sekunde mal nachjustieren
 */

#endif /* INC_ZCOMMUNICATION_H_ */
