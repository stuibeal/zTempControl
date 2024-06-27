/*
 * i2c_slave.c
 *
 *  Created on: Jun 21, 2024
 *      Author: al
 */
#include "i2c_slave.h"

uint8_t i2cRxBuffer[TEMP_RX_BYTES];
uint8_t i2cTxBuffer[TEMP_TX_BYTES];

int16_t blockAussenTemp = 0;
int16_t blockInnenTemp = 0;
uint8_t batterieStatus = 0;
uint8_t eingangsSpannung = 0;
uint16_t stromVerbrauchAktuell = 0;
uint16_t stromVerbrauchLetzteZapfung = 0;
uint16_t kuehlWasserFlowRate = 0;

uint8_t befehlVomMaster = 0;
int16_t zulaufTemp = 0;
int16_t kuehlWasserTemp = 0;
uint16_t befehlDaten = 0;


/**
 * @brief	Startet I2C HAL in Slave Mode DMA
 */
void i2c_slaveStartListen(I2C_HandleTypeDef *I2cHandle) {
	if (HAL_I2C_Slave_Receive_DMA(I2cHandle, i2cRxBuffer, 1) //TODO TEMP_RX_BYTES
			!= HAL_OK) {
		/* Transfer error in reception process */
		Error_Handler();
	}
}

void i2cRxDataConvert(void) {
	befehlVomMaster = i2cRxBuffer[0];
	zulaufTemp = (int16_t) (i2cRxBuffer[3] << 8) + i2cRxBuffer[2];
	kuehlWasserTemp = (int16_t) (i2cRxBuffer[5] << 8) + i2cRxBuffer[4];
	befehlDaten = (uint16_t) (i2cRxBuffer[7] << 8) + i2cRxBuffer[6];
}

void i2cTxDataConvert(void) {
	i2cTxBuffer[0] = blockAussenTemp & 0xff;
	i2cTxBuffer[1] = (blockAussenTemp >> 8) & 0xff;
	i2cTxBuffer[2] = blockInnenTemp & 0xff;
	i2cTxBuffer[3] = (blockInnenTemp >> 8) & 0xff;
	i2cTxBuffer[4] = batterieStatus;
	i2cTxBuffer[5] = eingangsSpannung;
	i2cTxBuffer[6] = stromVerbrauchAktuell & 0xff;
	i2cTxBuffer[7] = (stromVerbrauchAktuell >> 8) & 0xff;
	i2cTxBuffer[8] = stromVerbrauchLetzteZapfung & 0xff;
	i2cTxBuffer[9] = (stromVerbrauchLetzteZapfung >> 8) & 0xff;
	i2cTxBuffer[10] = kuehlWasserFlowRate & 0xff;
	i2cTxBuffer[11] = (kuehlWasserFlowRate >> 8) & 0xff;
}

/**
 * @brief  Tx Transfer completed callback.
 * @param  I2cHandle: I2C handle.
 * @note   Switcht vom Sender wieder zum Reciever
 * @retval None
 */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
	if (HAL_I2C_Slave_Receive_DMA(I2cHandle, i2cRxBuffer, 1) //TODO TEMP_RX_BYTES
			!= HAL_OK) {
		Error_Handler();
	}

}

/**
 * @brief  Rx Transfer completed callback.
 * @param  I2cHandle: I2C handle
 * @note   Schaltet nach Empfang wieder in Slave Mode
 *         kopiert die empfangenen Daten vom Buffer in das Register
 *         wenn man wissen will dass was empfangen wurde, einfach dataRxRegister[0] ansehen
 * @retval None
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {

//Nachdem Empfang komplett ist die Daten in die globalen Variablen schreiben
	i2cRxDataConvert();

	/* Jetzt verschick mal wieder Modus*/
	if (HAL_I2C_Slave_Transmit_DMA(&hi2c1, i2cTxBuffer, TEMP_TX_BYTES)
			!= HAL_OK) {
		Error_Handler();
	}
	/* kucken bis er wirklich in Slave Mode is */
	while (HAL_I2C_GetMode(I2cHandle) != HAL_I2C_MODE_SLAVE) {
	}
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//Nach Empfang die Bufferdaten in die Variablen schieben
	i2cTxDataConvert();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle) {
	//HAL_I2C_Slave_Receive_DMA(I2cHandle, i2cRxBuffer, TEMP_RX_BYTES);
	i2c_slaveStartListen(I2cHandle);
}

