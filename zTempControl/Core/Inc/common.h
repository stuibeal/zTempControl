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
#include "pid.h"

#define BATT_ULTRAHIGH 0x0
#define BATT_HIGH 0x01
#define BATT_NORMAL 0x02
#define BATT_LOW 0x03
#define BATT_ULTRALOW 0x04



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

extern modbusHandler_t ModbusH1;
extern uint16_t ModbusDATA1[16];
extern modbusHandler_t ModbusH2;
extern uint16_t ModbusDATA2[16];

//I2C
#define JUST_COMMUNICATE 0x01
#define SET_TEMPERATUR 0x02
#define EBI_MODE 0xF9    // don't know really know
#define BEGIN_ZAPF  0xFA  // Zapfprogramm beginnen
#define END_ZAPF  0xFB    // Zapfprogramm beenden, gezapfte Milliliter übertragen
#define KURZ_VOR_ZAPFENDE 0xFC // sagt das wir kurz vor Ende sind → Valve schließen -> PID auf konservativ
#define LOW_ENERGY = 0xFD // //LEDS nicht benutzen  byte3: 0: vollgas   1: sparen
#define WACH_AUF = 0xFE // alles wieder hochfahren und für den neuen Tag bereiten
#define ZAPFEN_STREICH 0xFF // alles runterfahren, licht aus, ton aus. Halt die Klappe, ich hab Feierabend.
// Vom Master
extern uint8_t befehlVomMaster;
extern int16_t zulaufTemp;
extern int16_t kuehlWasserTemp;
extern uint16_t befehlDaten;
// Zum Master
extern int16_t blockAussenTemp;
extern int16_t blockInnenTemp;
extern uint8_t batterieStatus;
extern uint8_t eingangsSpannung;
extern uint16_t stromVerbrauchAktuell;
extern uint16_t stromVerbrauchLetzteZapfung;
extern uint16_t kuehlWasserFlowRate;


// PT100 Temperatursensoren
extern float pt100AussenTemp;
extern float pt100InnenTemp;

//PID Regelung

extern PID_TypeDef innenPID;
extern PID_TypeDef aussenPID;
extern double innenPIDout;
extern double aussenPIDout;

//Flowsensor
extern uint16_t flowRate;
extern volatile uint32_t flowCounter;

//global, ist grad zapfen oder nicht
extern uint8_t zapfBool;




#endif /* INC_COMMON_H_ */
