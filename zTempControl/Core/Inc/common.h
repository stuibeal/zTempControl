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
