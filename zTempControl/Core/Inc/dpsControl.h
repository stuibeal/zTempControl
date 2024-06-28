/*
 * dpsControl.h
 *
 *  Created on: Jun 27, 2024
 *      Author: stuibeal
 *
 *      rxDPS2 PA3 // Kabel blau -> DPS2_TX (DPSkabel schwarz)
 *      txDPS2 PA2 //Kabel gelb -> DPS2_RX (DPSkabel gelb)
 *		rxDPS1 PA10  //Kabel grün -> DPS1_TX (DPSkabel gelb)
 *		txDPS1 PA9  //Kabel rot -> DPS1_RX (DPSkabel schwarz)
 *		DPS_baud 19200 //Speed of DPS (beim Einschalten V Taste gedrückt halten beim DPS5020)
 *
 *
 */

#ifndef INC_DPSCONTROL_H_
#define INC_DPSCONTROL_H_

#include "stdint.h"

//MODBUS
#define RD_U_SET 0x0
#define RD_I_SET 0x01
#define RD_UOUT 0x02
#define RD_IOUT 0x03
#define RD_POWER 0x04
#define RD_UIN 0x05
#define RD_KEYLOCK 0x06
#define RD_ONOFF 0x09
#define RD_BACKLIGHT 0x0A

/*  REGISTER RuiDeng DPS5020
 Modbus RTU

 Function   Description       Number of   Decimal   UNIT  Read/Write    Register
 bytes       places                        address
 U-SET      Voltage setting   2           2         V     R/W           0000H
 I-SET      Current setting   2           2         A     R/W           0001H
 UOUT       Output voltage    2           2         V     R             0002H
 display value
 IOUT       Output current    2           2         A     R             0003H
 display value
 POWER      Output power      2           1 or 2    W     R             0004H
 display value
 UIN        Input voltage     2           2         V     R             0005H
 display value
 LOCK       Key lock          2           0         -     R/W           0006H
 PROTECT    ProtectionStatus  2           0         -     R             0007H
 CV/CC      Status CV/CC      2           0         -     R             0008H
 ONOFF      Switch output     2           0         -     R/W           0009H
 B_LED      Backlight bright  2           0         -     R/W           000AH
 MODEL      Product Model     2           0         -     R             000BH
 VERSON     Firmware Version  2           0         -     R             000CH
 */

void startDPS(void);
uint16_t getEingangsSpannung(void);
uint16_t getWatt(void);
void dpsSetLock(uint16_t yesno);
void dpsSetBacklight(uint16_t backlightvalue);
void dpsOnOff(uint16_t onoff);
uint16_t getAusgangsSpannung(void);
uint16_t getAusgangsStrom(void);
void dpsSetCurrent(uint16_t ampere);
void dpsSetVoltage(uint16_t volt);
void dpsSetSingleRegister(uint8_t dpsRegister, uint16_t dpsData);
uint16_t dps0getSingleRegister(uint8_t dpsRegister);
uint16_t dps1getSingleRegister(uint8_t dpsRegister);
void dpsError(void);

#endif /* INC_DPSCONTROL_H_ */
